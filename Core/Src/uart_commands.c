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
typedef struct {
    uint8_t level;
    uint8_t motorStatus;
    char mode[12];
} StatusSnapshot;

static StatusSnapshot lastSent = {255, 255, "INIT"};

// simple string splitter (faster than strtok)
static char* next_token(char** ctx) {
    char* s = *ctx;
    if (!s) return NULL;
    char* colon = strchr(s, ':');
    if (colon) { *colon = '\0'; *ctx = colon + 1; }
    else { *ctx = NULL; }
    return s;
}

void UART_SendStatusPacket(void)
{
    extern ADC_Data adcData;
    extern volatile uint8_t motorStatus;
    extern volatile bool manualActive;
    extern volatile bool semiAutoActive;
    extern volatile bool timerActive;
    extern volatile bool searchActive;
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
    else if (searchActive)    mode = "SEARCH";
    else if (countdownActive) mode = "COUNTDOWN";
    else if (twistActive)     mode = "TWIST";

    bool stateChanged =
        (lastSent.level != submerged) ||
        (lastSent.motorStatus != motorStatus) ||
        (strcmp(lastSent.mode, mode) != 0);

    if (!stateChanged)
        return;   // nothing changed → don’t resend

    lastSent.level = submerged;
    lastSent.motorStatus = motorStatus;
    strncpy(lastSent.mode, mode, sizeof(lastSent.mode)-1);

    char buf[80];
    snprintf(buf, sizeof(buf),
             "STATUS:MOTOR:%s:LEVEL:%d:MODE:%s",
             motorStatus ? "ON" : "OFF",
             submerged, mode);

    UART_TransmitPacket(buf);
}

void UART_HandleCommand(const char *pkt)
{
    if (!pkt || !*pkt) return;

    char buf[UART_RX_BUFFER_SIZE];
    strncpy(buf, pkt, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    // remove wrapper chars
    if (buf[0] == '@') memmove(buf, buf+1, strlen(buf));
    char *end = strchr(buf, '#');
    if (end) *end = '\0';

    char *ctx = buf;
    char *cmd = next_token(&ctx);
    if (!cmd) return;

    /* --- Commands --- */
    if (!strcmp(cmd, "PING")) { ack("PONG"); return; }

    else if (!strcmp(cmd, "MANUAL")) {
        char *state = next_token(&ctx);
        if (!state) { err("PARAM"); return; }
        if (!strcmp(state,"ON"))  ModelHandle_ToggleManual();
        else if (!strcmp(state,"OFF")) ModelHandle_StopAllModesAndMotor();
        else { err("FORMAT"); return; }
        ack("MANUAL_OK");
    }

    else if (!strcmp(cmd, "TWIST")) {
        char *sub = next_token(&ctx);
        if (sub && !strcmp(sub,"SET")) {
            uint16_t on = atoi(next_token(&ctx));
            uint16_t off = atoi(next_token(&ctx));
            ModelHandle_StartTwist(on,off);
            ack("TWIST_OK");
        } else if (sub && !strcmp(sub,"STOP")) {
            ModelHandle_StopTwist();
            ack("TWIST_STOP");
        } else err("FORMAT");
    }

    else if (!strcmp(cmd, "SEARCH")) {
        char *sub = next_token(&ctx);
        if (sub && !strcmp(sub,"SET")) {
            uint16_t gap = atoi(next_token(&ctx));
            uint16_t probe = atoi(next_token(&ctx));
            ModelHandle_StartSearch(gap,probe);
            ack("SEARCH_OK");
        } else if (sub && !strcmp(sub,"STOP")) {
            ModelHandle_StopSearch();
            ack("SEARCH_STOP");
        } else err("FORMAT");
    }

    else if (!strcmp(cmd, "TIMER")) {
        char *sub = next_token(&ctx);
        if (sub && !strcmp(sub,"SET")) {
            uint8_t h1 = atoi(next_token(&ctx));
            uint8_t m1 = atoi(next_token(&ctx));
            uint8_t h2 = atoi(next_token(&ctx));
            uint8_t m2 = atoi(next_token(&ctx));
            timerSlots[0].active = true;
            timerSlots[0].onTimeSeconds  = ModelHandle_TimeToSeconds(h1,m1);
            timerSlots[0].offTimeSeconds = ModelHandle_TimeToSeconds(h2,m2);
            ack("TIMER_OK");
        } else if (sub && !strcmp(sub,"STOP")) {
            timerSlots[0].active = false;
            ModelHandle_StopAllModesAndMotor();
            ack("TIMER_STOP");
        } else err("FORMAT");
    }

    else if (!strcmp(cmd, "COUNTDOWN")) {
        char *sub = next_token(&ctx);
        if (sub && !strcmp(sub,"ON")) {
            uint16_t min = atoi(next_token(&ctx));
            if (!min) min = 1;
            ModelHandle_StartCountdown(min*60,1);
            ack("COUNTDOWN_ON");
        } else if (sub && !strcmp(sub,"OFF")) {
            ModelHandle_StopCountdown();
            ack("COUNTDOWN_OFF");
        } else err("FORMAT");
    }

    else if (!strcmp(cmd, "STATUS")) {
        // Force resend regardless of cache
        memset(&lastSent, 0xFF, sizeof(lastSent));
        static uint32_t lastStatusCheck = 0;
        if (HAL_GetTick() - lastStatusCheck > 5000) {
            UART_SendStatusPacket();
            lastStatusCheck = HAL_GetTick();
        }

    }
    else {
//        err("UNKNOWN");
        return;
    }

    g_screenUpdatePending = true;
}
