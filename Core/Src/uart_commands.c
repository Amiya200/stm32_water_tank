#include "uart_commands.h"
#include "uart.h"
#include "model_handle.h"
#include "relay.h"
#include "rtc_i2c.h"
#include <stdlib.h>
#include <string.h>

extern bool g_screenUpdatePending;
extern TimerSlot timerSlots[5];

static inline void ack(const char *msg) { UART_TransmitPacket(msg); }
static inline void err(const char *msg) { UART_TransmitPacket(msg); }

/* =========================
   Cached status report
   ========================= */
typedef struct {
    uint8_t level;
    uint8_t motorStatus;
    char mode[12];
} StatusSnapshot;

static StatusSnapshot lastSent = {255, 255, "INIT"};

/* Simple ':'-based tokenizer (no strtok) */
static char* next_token(char** ctx) {
    char* s = *ctx;
    if (!s) return NULL;
    char* colon = strchr(s, ':');
    if (colon) { *colon = '\0'; *ctx = colon + 1; }
    else { *ctx = NULL; }
    return s;
}

/* =========================
   STATUS PACKET
   ========================= */
void UART_SendStatusPacket(void)
{
    extern ADC_Data adcData;
    extern volatile uint8_t motorStatus;
    extern volatile bool manualActive;
    extern volatile bool semiAutoActive;
    extern volatile bool timerActive;
    extern volatile bool countdownActive;
    extern volatile bool twistActive;

    int submerged = 0;
    for (int i = 0; i < 5; i++) {
        if (adcData.voltages[i] < 0.1f) submerged++;
    }

    const char *mode = "IDLE";
    if (manualActive)         mode = "MANUAL";
    else if (semiAutoActive)  mode = "SEMIAUTO";
    else if (timerActive)     mode = "TIMER";
    else if (countdownActive) mode = "COUNTDOWN";
    else if (twistActive)     mode = "TWIST";

    bool changed =
        (lastSent.level != submerged) ||
        (lastSent.motorStatus != motorStatus) ||
        (strcmp(lastSent.mode, mode) != 0);

    if (!changed) return;

    lastSent.level = submerged;
    lastSent.motorStatus = motorStatus;
    strncpy(lastSent.mode, mode, sizeof(lastSent.mode) - 1);

    char buf[80];
    snprintf(buf, sizeof(buf),
             "STATUS:MOTOR:%s:LEVEL:%d:MODE:%s",
             motorStatus ? "ON" : "OFF",
             submerged, mode);

    UART_TransmitPacket(buf);
}

/* =========================
   COMMAND HANDLER
   ========================= */
void UART_HandleCommand(const char *pkt)
{
    if (!pkt || !*pkt) return;

    char buf[UART_RX_BUFFER_SIZE];
    strncpy(buf, pkt, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    /* remove wrappers like '@' and '#' */
    if (buf[0] == '@') memmove(buf, buf + 1, strlen(buf));
    char *end = strchr(buf, '#');
    if (end) *end = '\0';

    char *ctx = buf;
    char *cmd = next_token(&ctx);
    if (!cmd) return;

    /* ---- BASIC ---- */
    if (!strcmp(cmd, "PING")) { ack("PONG"); return; }

    /* ---- MANUAL ---- */
    else if (!strcmp(cmd, "MANUAL")) {
        char *state = next_token(&ctx);
        if (!state) { err("PARAM"); return; }
        if (!strcmp(state, "ON"))  ModelHandle_ToggleManual();
        else if (!strcmp(state, "OFF")) ModelHandle_StopAllModesAndMotor();
        else { err("FORMAT"); return; }
        ack("MANUAL_OK");
    }
    /* ---- AUTO ---- */
    else if (!strcmp(cmd, "AUTO"))
    {
        char *state = next_token(&ctx);
        if (!state) { err("PARAM"); return; }

        if (!strcmp(state, "ON"))
        {
//            clear_all_modes();
            autoActive = true;
//            start_motor();      // Auto mode immediately runs dry-run FSM
            ack("AUTO_ON");
        }
        else if (!strcmp(state, "OFF"))
        {
            autoActive = false;
//            stop_motor();
            ack("AUTO_OFF");
        }
        else
        {
            err("FORMAT");
        }
    }


    /* ---- TWIST ---- */
    else if (!strcmp(cmd, "TWIST"))
    {
        char *sub = next_token(&ctx);

        if (sub && !strcmp(sub, "SET"))
        {
            uint16_t onDur  = atoi(next_token(&ctx));
            uint16_t offDur = atoi(next_token(&ctx));

            uint8_t onH  = atoi(next_token(&ctx));
            uint8_t onM  = atoi(next_token(&ctx));
            uint8_t offH = atoi(next_token(&ctx));
            uint8_t offM = atoi(next_token(&ctx));

            // Ignore days (skip tokens until NULL)
            while (next_token(&ctx) != NULL);

            ModelHandle_StartTwist(onDur, offDur, onH, onM, offH, offM);
            ack("TWIST_OK");
        }
        else if (sub && !strcmp(sub, "STOP"))
        {
            ModelHandle_StopTwist();
            ack("TWIST_STOP");
        }
        else
        {
            err("FORMAT");
        }
    }

    /* ---- TIMER (multi-slot support) ---- */
    else if (!strcmp(cmd, "TIMER")) {
        char *sub = next_token(&ctx);
        if (sub && !strcmp(sub, "SET")) {
            uint8_t slotIndex = 0;
            bool ok = true;

            // Loop through multiple timer packets in the input buffer
            while (slotIndex < 5) {
                char *h1s = next_token(&ctx);
                char *m1s = next_token(&ctx);
                char *h2s = next_token(&ctx);
                char *m2s = next_token(&ctx);

                if (!h1s || !m1s || !h2s || !m2s)
                    break; // no more slots

                int h1 = atoi(h1s);
                int m1 = atoi(m1s);
                int h2 = atoi(h2s);
                int m2 = atoi(m2s);

                if (h1 < 0 || h1 > 23 || h2 < 0 || h2 > 23 ||
                    m1 < 0 || m1 > 59 || m2 < 0 || m2 > 59)
                {
                    ok = false;
                    break;
                }

                timerSlots[slotIndex].enabled = true;
                timerSlots[slotIndex].onHour  = h1;
                timerSlots[slotIndex].onMinute = m1;
                timerSlots[slotIndex].offHour  = h2;
                timerSlots[slotIndex].offMinute = m2;
                slotIndex++;
            }

            // Disable remaining slots
            for (; slotIndex < 5; slotIndex++) {
                timerSlots[slotIndex].enabled = false;
            }

            if (ok){
            	ack("TIMER_OK");
            	ModelHandle_StartTimer();
            }


            else
                err("TIMER_FORMAT");
        }

        else if (sub && !strcmp(sub, "STOP")) {
            for (int i=0; i<5; i++)
                timerSlots[i].enabled = false;
            ModelHandle_StopAllModesAndMotor();
            ack("TIMER_STOP");
        }

        else err("FORMAT");
    }


    /* ---- SEMIAUTO ---- */
    else if (!strcmp(cmd, "SEMIAUTO")) {
        char *sub = next_token(&ctx);
        if (sub && !strcmp(sub, "ON")) {
            ModelHandle_StartSemiAuto();
            ack("SEMIAUTO_ON");
        }
        else if (sub && !strcmp(sub, "OFF")) {
            ModelHandle_StopAllModesAndMotor();
            ack("SEMIAUTO_OFF");
        }
        else err("FORMAT");
        g_screenUpdatePending = true;
        return;
    }

    /* ---- COUNTDOWN ---- */
    else if (!strcmp(cmd, "COUNTDOWN")) {
        char *sub = next_token(&ctx);
        if (sub && !strcmp(sub, "ON")) {
            uint16_t min = atoi(next_token(&ctx));
            if (!min) min = 1;
            ModelHandle_StartCountdown(min * 60, 1);
            ack("COUNTDOWN_ON");
        } else if (sub && !strcmp(sub, "OFF")) {
            ModelHandle_StopCountdown();
            ack("COUNTDOWN_OFF");
        } else err("FORMAT");
    }

    /* ---- STATUS ---- */
    else if (!strcmp(cmd, "STATUS")) {
        static uint32_t lastReply = 0;
        uint32_t now = HAL_GetTick();
        if (now - lastReply >= 5000) {   // reply only once every 5 s
            UART_SendStatusPacket();
            lastReply = now;
        }
        return;
    }

    else {
//        err("UNKNOWN");
        return;
    }

    g_screenUpdatePending = true;
}
