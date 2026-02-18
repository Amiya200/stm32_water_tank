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

/* =========================================================
   STATUS CACHE
========================================================= */

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

/* =========================================================
   TOKENIZER
========================================================= */

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

/* =========================================================
   STATUS PACKET
========================================================= */

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

    char buf[120];
    snprintf(buf, sizeof(buf),
             "@STATUS:MOTOR:%s:LEVEL:%d:MODE:%s#",
             motorStatus ? "ON" : "OFF",
             level,
             mode);

    UART_TransmitPacket(buf);
}

/* =========================================================
   COMMAND HANDLER
========================================================= */

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

    /* ================= PING ================= */
    if (!strcmp(cmd, "PING"))
    {
        ack("@PONG#");
        return;
    }

    /* ================= RESTART ================= */
    else if (!strcmp(cmd, "RESTART"))
    {
        char *sub = next_token(&ctx);

        if (sub && !strcmp(sub, "ON"))
        {
            ModelHandle_StartRestart();
            ack("@RESTART_ON#");
        }
        else if (sub && !strcmp(sub, "OFF"))
        {
            ModelHandle_StopRestart();
            ack("@RESTART_OFF#");
        }
        else err("@FORMAT#");
    }

    /* ================= MANUAL ================= */
    else if (!strcmp(cmd, "MANUAL"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }

        if (!strcmp(state, "ON"))
        {
            if (!ModelHandle_IsManualActive())
                ModelHandle_ToggleManual();
            ack("@MANUAL_ON#");
        }
        else if (!strcmp(state, "OFF"))
        {
            if (ModelHandle_IsManualActive())
                ModelHandle_ToggleManual();
            ack("@MANUAL_OFF#");
        }
        else err("@FORMAT#");
    }

    /* ================= SEMIAUTO ================= */
    else if (!strcmp(cmd, "SEMIAUTO"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }

        if (!strcmp(state, "ON"))
        {
            ModelHandle_StartSemiAuto();
            ack("@SEMIAUTO_ON#");
        }
        else if (!strcmp(state, "OFF"))
        {
            ModelHandle_StopSemiAuto();
            ack("@SEMIAUTO_OFF#");
        }
        else err("@FORMAT#");
    }

    /* ================= AUTO ================= */
    else if (!strcmp(cmd, "AUTO"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("@FORMAT#"); return; }

        if (!strcmp(state, "ON"))
        {
            ModelHandle_StartAuto(
                ModelHandle_GetAutoGap(),
                ModelHandle_GetAutoMaxRun(),
                ModelHandle_GetAutoRetry()
            );
            ack("@AUTO_ON#");
        }
        else if (!strcmp(state, "OFF"))
        {
            ModelHandle_StopAuto();
            ack("@AUTO_OFF#");
        }
        else err("@FORMAT#");
    }
    else if (!strcmp(cmd, "SETTINGS"))
    {
        if (!ctx) return;

        uint32_t dryRun  = ModelHandle_GetGapTime();
        uint8_t  retry   = ModelHandle_GetRetryCount();
        uint16_t maxRun  = ModelHandle_GetMaxRunTime();
        uint16_t lowV    = ModelHandle_GetUnderVolt();
        uint16_t highV   = ModelHandle_GetOverVolt();
        int16_t  overL   = (int16_t)ModelHandle_GetOverloadLimit();
        int16_t  underL  = (int16_t)ModelHandle_GetUnderloadLimit();
        uint8_t  pwrRes  = ModelHandle_GetPowerRestoreMode();

        char *saveptr;
        char *pair = strtok_r(ctx, ";", &saveptr);

        while (pair)
        {
            char *eq = strchr(pair, '=');
            if (eq)
            {
                *eq = '\0';
                char *key = pair;
                char *val = eq + 1;

                if (!strcmp(key, "dryRunGap"))
                {
                    uint32_t min = atoi(val);
                    dryRun = min * 60UL;
                }
                else if (!strcmp(key, "testingGap"))
                    retry = atoi(val);

                else if (!strcmp(key, "maxRun"))
                    maxRun = atoi(val);

                else if (!strcmp(key, "lowVolt"))
                    lowV = atoi(val);

                else if (!strcmp(key, "highVolt"))
                    highV = atoi(val);

                else if (!strcmp(key, "overLoad"))
                    overL = atoi(val);

                else if (!strcmp(key, "underLoad"))
                    underL = atoi(val);

                else if (!strcmp(key, "powerRestore"))
                    pwrRes = atoi(val);
            }

            pair = strtok_r(NULL, ";", &saveptr);
        }

        ModelHandle_SetUserSettings(
            dryRun,
            retry,
            lowV,
            highV,
            overL,
            underL,
            maxRun
        );

        ModelHandle_SetPowerRestoreMode(pwrRes);

        ack("@SETTINGS_OK#");
    }

    else if (!strcmp(cmd, "COUNTDOWN"))
    {
        char *sub = next_token(&ctx);

        if (sub && !strcmp(sub, "ON"))
        {
            char *minStr = next_token(&ctx);
            if (!minStr) { err("@FORMAT#"); return; }

            uint32_t min = atoi(minStr);
            if (min == 0) min = 1;

            ModelHandle_StartCountdown(min * 60UL);
            ack("@COUNTDOWN_ON#");
        }
        else if (sub && !strcmp(sub, "OFF"))
        {
            ModelHandle_StopCountdown();
            ack("@COUNTDOWN_OFF#");
        }
        else err("@FORMAT#");
    }

    /* ================= TIMER ================= */
    else if (!strcmp(cmd, "TIMER"))
    {
        char *sub = next_token(&ctx);

        if (sub && !strcmp(sub, "STOP"))
        {
            ModelHandle_StopTimer();
            ack("@TIMER_STOP#");
            return;
        }

        if (sub && !strcmp(sub, "SET"))
        {
            char *slotStr = next_token(&ctx);
            char *daysStr = next_token(&ctx);
            char *h1s = next_token(&ctx);
            char *m1s = next_token(&ctx);
            char *h2s = next_token(&ctx);
            char *m2s = next_token(&ctx);
            char *gapStr = next_token(&ctx);
            char *extra = next_token(&ctx);   // <-- NEW (handles extra field)

            if (!slotStr || !daysStr || !h1s || !m1s || !h2s || !m2s || !gapStr)
            {
                err("@TIMER_FORMAT#");
                return;
            }

            if (extra != NULL)   // If extra token exists, shift gap
            {
                gapStr = extra;
            }

            int slot = atoi(slotStr);
            if (slot < 1 || slot > 5)
            {
                err("@TIMER_RANGE#");
                return;
            }
            uint8_t idx = slot - 1;

            timerSlots[idx].enabled = true;
            timerSlots[idx].dayMask = 0x7F;
            timerSlots[idx].onHour = atoi(h1s);
            timerSlots[idx].onMinute = atoi(m1s);
            timerSlots[idx].offHour = atoi(h2s);
            timerSlots[idx].offMinute = atoi(m2s);
            timerSlots[idx].gapMinutes = atoi(gapStr);

            ModelHandle_StartTimer();
            ack("@TIMER_OK#");
        }
        else err("@FORMAT#");
    }
    else if (!strcmp(cmd, "STATUS"))
    {
        UART_SendStatusPacket();
        return;
    }

    g_screenUpdatePending = true;
}
