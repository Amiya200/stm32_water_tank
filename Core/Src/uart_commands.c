#include "uart_commands.h"
#include "uart.h"
#include "model_handle.h"
#include "relay.h"
#include "rtc_i2c.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

extern bool g_screenUpdatePending;
extern TimerSlot timerSlots[5];

static inline void ack(const char *msg)
{
    UART_TransmitPacket(msg);
}

static inline void err(const char *msg)
{
    UART_TransmitPacket(msg);
}

typedef struct {
    uint8_t level;
    uint8_t motorStatus;
    char    mode[12];
} StatusSnapshot;

static StatusSnapshot lastSent = {255, 255, "INIT"};

void UART_InitCommandSystem(void)
{
    lastSent.level = 255;
    lastSent.motorStatus = 255;
    strcpy(lastSent.mode, "INIT");
}

static uint8_t parseDays(const char *daysStr)
{
    uint8_t mask = 0;
    if (strstr(daysStr, "mon")) mask |= (1 << 0);
    if (strstr(daysStr, "tue")) mask |= (1 << 1);
    if (strstr(daysStr, "wed")) mask |= (1 << 2);
    if (strstr(daysStr, "thu")) mask |= (1 << 3);
    if (strstr(daysStr, "fri")) mask |= (1 << 4);
    if (strstr(daysStr, "sat")) mask |= (1 << 5);
    if (strstr(daysStr, "sun")) mask |= (1 << 6);
    return mask;
}

static char* next_token(char** ctx)
{
    char* s = *ctx;
    if (!s) return NULL;

    char* colon = strchr(s, ':');
    if (colon)
    {
        *colon = '\0';
        *ctx = colon + 1;
    }
    else
    {
        *ctx = NULL;
    }
    return s;
}

void UART_SendStatusPacket(void)
{
    extern volatile uint8_t motorStatus;

    uint8_t level = ModelHandle_GetTankLevelPercent();
    const char *mode = "IDLE";

    if (ModelHandle_IsRestartActive()) mode = "RESTART";
    else if (ModelHandle_IsManualActive()) mode = "MANUAL";
    else if (ModelHandle_IsAutoActive()) mode = "AUTO";
    else if (timerSlots[0].enabled ||
             timerSlots[1].enabled ||
             timerSlots[2].enabled ||
             timerSlots[3].enabled ||
             timerSlots[4].enabled)
        mode = "TIMER";

    bool changed =
        (lastSent.level != level) ||
        (lastSent.motorStatus != motorStatus) ||
        (strcmp(lastSent.mode, mode) != 0);

    if (!changed) return;

    lastSent.level = level;
    lastSent.motorStatus = motorStatus;
    strncpy(lastSent.mode, mode, sizeof(lastSent.mode) - 1);
    lastSent.mode[sizeof(lastSent.mode) - 1] = '\0';

    char buf[100];
    snprintf(buf, sizeof(buf),
             "STATUS:MOTOR:%s:LEVEL:%d:MODE:%s",
             motorStatus ? "ON" : "OFF",
             level,
             mode);

    UART_TransmitPacket(buf);
}

void UART_HandleCommand(const char *pkt)
{
    if (!pkt || !*pkt) return;

    char buf[UART_RX_BUFFER_SIZE];
    strncpy(buf, pkt, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    if (buf[0] == '@')
        memmove(buf, buf + 1, strlen(buf));

    char *end = strchr(buf, '#');
    if (end) *end = '\0';

    char *ctx = buf;
    char *cmd = next_token(&ctx);
    if (!cmd) return;

    if (!strcmp(cmd, "PING"))
    {
        ack("PONG");
        return;
    }

    else if (!strcmp(cmd, "RESTART"))
    {
        char *sub = next_token(&ctx);
        if (sub && !strcmp(sub, "ON"))
        {
            ModelHandle_StartRestart();
            ack("RESTART_ON");
        }
        else if (sub && !strcmp(sub, "OFF"))
        {
            ModelHandle_StopRestart();
            ack("RESTART_OFF");
        }
        else err("FORMAT");
    }

    else if (!strcmp(cmd, "MANUAL"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("FORMAT"); return; }

        if (!strcmp(state, "ON"))
        {
            ModelHandle_StopAllModesAndMotor();
            ModelHandle_ToggleManual();
            ack("MANUAL_ON");
        }
        else if (!strcmp(state, "OFF"))
        {
            ModelHandle_StopAllModesAndMotor();
            ack("MANUAL_OFF");
        }
        else err("FORMAT");
    }

    else if (!strcmp(cmd, "SEMIAUTO"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("FORMAT"); return; }

        if (!strcmp(state, "ON"))
        {
            ModelHandle_StopAllModesAndMotor();
            ModelHandle_StartSemiAuto();
            ack("SEMIAUTO_ON");
        }
        else if (!strcmp(state, "OFF"))
        {
            ModelHandle_StopAllModesAndMotor();
            ack("SEMIAUTO_OFF");
        }
        else err("FORMAT");
    }

    else if (!strcmp(cmd, "AUTO"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("FORMAT"); return; }

        if (!strcmp(state, "ON"))
        {
            ModelHandle_StartAuto(
                ModelHandle_GetAutoGap(),
                ModelHandle_GetAutoMaxRun(),
                ModelHandle_GetAutoRetry()
            );
            ack("AUTO_ON");
        }
        else if (!strcmp(state, "OFF"))
        {
            ModelHandle_StopAuto();
            ack("AUTO_OFF");
        }
        else err("FORMAT");
    }

    else if (!strcmp(cmd, "TIMER"))
    {
        char *sub = next_token(&ctx);

        if (sub && !strcmp(sub, "SET"))
        {
            char *slotStr = next_token(&ctx);
            char *daysStr = next_token(&ctx);
            char *h1s = next_token(&ctx);
            char *m1s = next_token(&ctx);
            char *h2s = next_token(&ctx);
            char *m2s = next_token(&ctx);
            char *gapStr = next_token(&ctx);

            if (!slotStr || !daysStr || !h1s || !m1s || !h2s || !m2s || !gapStr)
            {
                err("TIMER_FORMAT");
                return;
            }

            int slot = atoi(slotStr);
            int h1 = atoi(h1s);
            int m1 = atoi(m1s);
            int h2 = atoi(h2s);
            int m2 = atoi(m2s);
            int gap = atoi(gapStr);

            if (slot < 1 || slot > 5 ||
                h1 < 0 || h1 > 23 ||
                m1 < 0 || m1 > 59 ||
                h2 < 0 || h2 > 23 ||
                m2 < 0 || m2 > 59 ||
                gap < 0 || gap > 60)
            {
                err("TIMER_RANGE");
                return;
            }

            uint8_t idx = slot - 1;

            timerSlots[idx].enabled = true;
            timerSlots[idx].dayMask = parseDays(daysStr);
            timerSlots[idx].onHour = h1;
            timerSlots[idx].onMinute = m1;
            timerSlots[idx].offHour = h2;
            timerSlots[idx].offMinute = m2;
            timerSlots[idx].gapMinutes = gap;

            ModelHandle_StartTimer();
            ack("TIMER_OK");
        }
        else if (sub && !strcmp(sub, "STOP"))
        {
            ModelHandle_StopTimer();
            ack("TIMER_STOP");
        }
        else err("FORMAT");
    }

    else if (!strcmp(cmd, "COUNTDOWN"))
    {
        char *sub = next_token(&ctx);

        if (sub && !strcmp(sub, "ON"))
        {
            char *minStr = next_token(&ctx);
            if (!minStr) { err("FORMAT"); return; }

            uint32_t min = atoi(minStr);
            if (min == 0) min = 1;

            ModelHandle_StartCountdown(min * 60UL);
            ack("COUNTDOWN_ON");
        }
        else if (sub && !strcmp(sub, "OFF"))
        {
            ModelHandle_StopCountdown();
            ack("COUNTDOWN_OFF");
        }
        else err("FORMAT");
    }

    else if (!strcmp(cmd, "STATUS"))
    {
        UART_SendStatusPacket();
        return;
    }

    g_screenUpdatePending = true;
}
