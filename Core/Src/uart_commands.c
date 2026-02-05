#include "uart_commands.h"
#include "uart.h"
#include "model_handle.h"
#include "relay.h"
#include "rtc_i2c.h"
#include "stm32f1xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

extern bool g_screenUpdatePending;
extern TimerSlot timerSlots[5];

static inline void ack(const char *msg) { UART_TransmitPacket(msg); }
static inline void err(const char *msg) { UART_TransmitPacket(msg); }

/* ---------------------- Helpers ---------------------- */

static void trim_newline(char *s)
{
    if (!s) return;
    s[strcspn(s, "\r\n")] = 0;
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

static uint8_t parseDays(const char *daysStr)
{
    uint8_t mask = 0;
    if (!daysStr) return 0;

    if (strstr(daysStr, "mon")) mask |= (1 << 0);
    if (strstr(daysStr, "tue")) mask |= (1 << 1);
    if (strstr(daysStr, "wed")) mask |= (1 << 2);
    if (strstr(daysStr, "thu")) mask |= (1 << 3);
    if (strstr(daysStr, "fri")) mask |= (1 << 4);
    if (strstr(daysStr, "sat")) mask |= (1 << 5);
    if (strstr(daysStr, "sun")) mask |= (1 << 6);

    return mask;
}

/* ---------------------- STATUS ---------------------- */

typedef struct {
    uint8_t level;
    uint8_t motorStatus;
    char mode[12];
} StatusSnapshot;

static StatusSnapshot lastSent = {255, 255, "INIT"};

void UART_SendStatusPacket(void)
{
    extern ADC_Data adcData;
    extern volatile uint8_t motorStatus;
    extern volatile bool manualActive;
    extern volatile bool semiAutoActive;
    extern volatile bool timerActive;
    extern volatile bool countdownActive;
    extern volatile bool twistActive;
    extern volatile bool autoActive;

    int submerged = 0;
    for (int i = 0; i < 4; i++)
    {
        if (adcData.voltages[i] < 0.1f)
            submerged++;
    }

    const char *mode = "IDLE";

    if (manualActive)         mode = "MANUAL";
    else if (semiAutoActive)  mode = "SEMIAUTO";
    else if (timerActive)     mode = "TIMER";
    else if (countdownActive) mode = "COUNTDOWN";
    else if (twistActive)     mode = "TWIST";
    else if (autoActive)      mode = "AUTO";

    bool changed =
        (lastSent.level != submerged) ||
        (lastSent.motorStatus != motorStatus) ||
        (strcmp(lastSent.mode, mode) != 0);

    if (!changed) return;

    lastSent.level = submerged;
    lastSent.motorStatus = motorStatus;
    strncpy(lastSent.mode, mode, sizeof(lastSent.mode) - 1);
    lastSent.mode[sizeof(lastSent.mode)-1] = '\0';

    char buf[80];
    snprintf(buf, sizeof(buf),
             "STATUS:MOTOR:%s:LEVEL:%d:MODE:%s",
             motorStatus ? "ON" : "OFF",
             submerged, mode);

    UART_TransmitPacket(buf);
}

/* ---------------------- COMMAND HANDLER ---------------------- */

void UART_HandleCommand(const char *pkt)
{
    if (!pkt || !*pkt) return;

    char buf[128];
    strncpy(buf, pkt, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    if (buf[0] == '@')
        memmove(buf, buf + 1, strlen(buf));

    char *end = strchr(buf, '#');
    if (end) *end = '\0';

    char *ctx = buf;
    char *cmd = next_token(&ctx);
    if (!cmd) return;

    trim_newline(cmd);

    /* ---------------- PING ---------------- */
    if (!strcmp(cmd, "PING"))
    {
        ack("PONG");
        return;
    }

    /* ---------------- MANUAL ---------------- */
    else if (!strcmp(cmd, "MANUAL"))
    {
        char *state = next_token(&ctx);
        trim_newline(state);

        if (!state) { err("FORMAT"); return; }

        if (!strcmp(state, "ON"))
            ModelHandle_ToggleManual();
        else if (!strcmp(state, "OFF"))
            ModelHandle_StopAllModesAndMotor();
        else { err("FORMAT"); return; }

        ack("MANUAL_OK");
    }

    /* ---------------- AUTO ---------------- */
    else if (!strcmp(cmd, "AUTO"))
    {
        char *sub = next_token(&ctx);
        trim_newline(sub);

        if (!sub) { err("AUTO_FORMAT"); return; }

        if (!strcmp(sub, "ON"))
        {
        	if (!ModelHandle_IsAutoActive()){
                ModelHandle_StartAuto(
                    ModelHandle_GetGapTime(),
                    ModelHandle_GetMaxRunTime(),
                    ModelHandle_GetRetryCount()
                );
                ack("AUTO_ON");
        	}
     }
        else if (!strcmp(sub, "OFF"))
        {
            ModelHandle_StopAuto();
            ack("AUTO_OFF");
        }
        else if (!strcmp(sub, "SET"))
        {
            char *gapStr   = next_token(&ctx);
            char *maxStr   = next_token(&ctx);
            char *retryStr = next_token(&ctx);

            if (!gapStr || !maxStr || !retryStr)
            {
                err("AUTO_SET_FORMAT");
                return;
            }

            uint16_t gap   = atoi(gapStr);
            uint16_t max   = atoi(maxStr);
            uint8_t  retry = atoi(retryStr);

            ModelHandle_SetAutoSettings(gap, max, retry);
            ack("AUTO_SET_OK");
        }
        else
        {
            err("AUTO_FORMAT");
        }
    }

    /* ---------------- TIMER ---------------- */
    else if (!strcmp(cmd, "TIMER"))
    {
        char *sub = next_token(&ctx);
        trim_newline(sub);

        if (sub && !strcmp(sub, "SET"))
        {
            char *slotStr   = next_token(&ctx);
            char *daysStr   = next_token(&ctx);
            char *h1s       = next_token(&ctx);
            char *m1s       = next_token(&ctx);
            char *h2s       = next_token(&ctx);
            char *m2s       = next_token(&ctx);
            char *enableStr = next_token(&ctx);
            char *gapStr    = next_token(&ctx);

            if (!slotStr || !daysStr || !h1s || !m1s ||
                !h2s || !m2s || !enableStr || !gapStr)
            {
                err("TIMER_FORMAT");
                return;
            }

            int slot = atoi(slotStr);
            if (slot < 1 || slot > 5)
            {
                err("TIMER_RANGE");
                return;
            }

            uint8_t idx = slot - 1;

            timerSlots[idx].enabled    = atoi(enableStr);
            timerSlots[idx].dayMask    = parseDays(daysStr);
            timerSlots[idx].onHour     = atoi(h1s);
            timerSlots[idx].onMinute   = atoi(m1s);
            timerSlots[idx].offHour    = atoi(h2s);
            timerSlots[idx].offMinute  = atoi(m2s);
            timerSlots[idx].gapMinutes = atoi(gapStr);

            ModelHandle_StartTimer();
            ack("TIMER_OK");
        }
        else if (sub && !strcmp(sub, "STOP"))
        {
            for (int i = 0; i < 5; i++)
                timerSlots[i].enabled = 0;

            ModelHandle_StopTimer();
            ack("TIMER_STOP");
        }
        else
        {
            err("FORMAT");
        }
    }

    /* ---------------- SEMIAUTO ---------------- */
    else if (!strcmp(cmd, "SEMIAUTO"))
    {
        char *sub = next_token(&ctx);
        trim_newline(sub);

        if (sub && !strcmp(sub, "ON"))
        {
            ModelHandle_StartSemiAuto();
            ack("SEMIAUTO_ON");
        }
        else if (sub && !strcmp(sub, "OFF"))
        {
            ModelHandle_StopSemiAuto();
            ack("SEMIAUTO_OFF");
        }
        else err("FORMAT");
    }

    /* ---------------- COUNTDOWN ---------------- */
    else if (!strcmp(cmd, "COUNTDOWN"))
    {
        char *sub = next_token(&ctx);
        trim_newline(sub);

        if (sub && !strcmp(sub, "ON"))
        {
            char *minStr = next_token(&ctx);
            uint32_t min = minStr ? atoi(minStr) : 1;
            if (min == 0) min = 1;

            ModelHandle_StartCountdown(min * 60);
            ack("COUNTDOWN_ON");
        }
        else if (sub && !strcmp(sub, "OFF"))
        {
            ModelHandle_StopCountdown();
            ack("COUNTDOWN_OFF");
        }
        else err("FORMAT");
    }

    /* ---------------- STATUS ---------------- */
    else if (!strcmp(cmd, "STATUS"))
    {
        UART_SendStatusPacket();
        return;
    }

    else
    {
        err("UNKNOWN");
        return;
    }

    g_screenUpdatePending = true;
}
