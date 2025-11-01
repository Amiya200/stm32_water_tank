#include "uart_commands.h"
#include "uart.h"
#include "model_handle.h"
#include "adc.h"
#include "relay.h"
#include "rtc_i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

/* ============================================================
   Helper transmit functions
   ============================================================ */
static inline void send_ack(const char *msg) {
    char buf[64];
    snprintf(buf, sizeof(buf), "@ACK:%s#\r\n", msg);
    UART_TransmitString(&huart1, buf);
}

static inline void send_error(const char *msg) {
    char buf[64];
    snprintf(buf, sizeof(buf), "@ERR:%s#\r\n", msg);
    UART_TransmitString(&huart1, buf);
}

/* ============================================================
   Status helpers
   ============================================================ */
void UART_SendStatusPacket(void)
{
    extern ADC_Data adcData;
    int submerged = 0;
    for (int i = 0; i < 5; i++) {
        if (adcData.voltages[i] < 0.1f) submerged++;
    }

    const char *motorState = motorStatus ? "ON" : "OFF";
    char buf[64];
    snprintf(buf, sizeof(buf),
             "@STATUS:MOTOR:%s:LEVEL:%d#\r\n",
             motorState, submerged);
    UART_TransmitString(&huart1, buf);
}

void UART_SendDryAlert(void) {
    UART_TransmitString(&huart1, "@ALERT:DRYRUN#\r\n");
}

/* ============================================================
   Command execution
   ============================================================ */
void UART_HandleCommand(const char *packet)
{
    if (!packet || packet[0] != '@')
        return;

    /* Copy safely to local buffer */
    char cmd[UART_RX_BUFFER_SIZE];
    strncpy(cmd, packet, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';

    /* Trim start/end markers */
    size_t len = strlen(cmd);
    if (cmd[len - 1] == '#') cmd[len - 1] = '\0';
    if (cmd[0] == '@') memmove(cmd, cmd + 1, len);

    /* Tokenize */
    char *saveptr;
    char *mainTok = strtok_r(cmd, ":", &saveptr);
    if (!mainTok) return;

    /* -------- MANUAL -------- */
    if (strcmp(mainTok, "MANUAL") == 0) {
        char *state = strtok_r(NULL, ":", &saveptr);
        if (!state) { send_error("MANUAL_PARAM"); return; }

        if (strcmp(state, "ON") == 0) {
            ModelHandle_StopAllModesAndMotor();
            ModelHandle_ToggleManual();
            send_ack("MANUAL_ON");
        } else if (strcmp(state, "OFF") == 0) {
            ModelHandle_StopAllModesAndMotor();
            send_ack("MANUAL_OFF");
        } else send_error("MANUAL_FORMAT");
    }

    /* -------- SEMI AUTO -------- */
    else if (strcmp(mainTok, "SEMI") == 0) {
        char *state = strtok_r(NULL, ":", &saveptr);
        if (!state) { send_error("SEMI_PARAM"); return; }

        if (strcmp(state, "ON") == 0) {
            ModelHandle_ResetAll();
            semiAutoActive = true;
            Relay_Set(1, true);
            send_ack("SEMI_ON");
        } else if (strcmp(state, "OFF") == 0) {
            ModelHandle_StopAllModesAndMotor();
            send_ack("SEMI_OFF");
        }
    }

    /* -------- COUNTDOWN -------- */
    else if (strcmp(mainTok, "COUNTDOWN") == 0) {
        char *state = strtok_r(NULL, ":", &saveptr);
        if (!state) { send_error("COUNT_PARAM"); return; }

        if (strcmp(state, "ON") == 0) {
            char *minStr = strtok_r(NULL, ":", &saveptr);
            uint16_t mins = (minStr) ? atoi(minStr) : 1;
            ModelHandle_StartCountdown(mins * 60, 1);
            send_ack("COUNTDOWN_ON");
        } else if (strcmp(state, "OFF") == 0 || strcmp(state, "DONE") == 0) {
            ModelHandle_StopCountdown();
            send_ack("COUNTDOWN_OFF");
        }
    }

    /* -------- TWIST -------- */
    else if (strcmp(mainTok, "TWIST") == 0) {
        char *sub = strtok_r(NULL, ":", &saveptr);
        if (sub && strcmp(sub, "SET") == 0) {
            char *onStr = strtok_r(NULL, ":", &saveptr);
            char *offStr = strtok_r(NULL, ":", &saveptr);
            if (onStr && offStr) {
                uint16_t on = atoi(onStr);
                uint16_t off = atoi(offStr);
                ModelHandle_StartTwist(on, off);
                send_ack("TWIST_SET");
            } else send_error("TWIST_ARGS");
        } else send_error("TWIST_FORMAT");
    }

    /* -------- SEARCH -------- */
    else if (strcmp(mainTok, "SEARCH") == 0) {
        char *sub = strtok_r(NULL, ":", &saveptr);
        if (sub && strcmp(sub, "SET") == 0) {
            char *gapStr = strtok_r(NULL, ":", &saveptr);
            char *dryStr = strtok_r(NULL, ":", &saveptr);
            if (gapStr && dryStr) {
                uint16_t gap = atoi(gapStr);
                uint16_t dry = atoi(dryStr);
                searchSettings.testingGapSeconds = gap;
                searchSettings.dryRunTimeSeconds = dry;
                searchSettings.searchActive = true;
                searchActive = true;
                send_ack("SEARCH_SET");
            } else send_error("SEARCH_ARGS");
        }
    }

    /* -------- TIMER -------- */
    else if (strcmp(mainTok, "TIMER") == 0) {
        char *sub = strtok_r(NULL, ":", &saveptr);
        if (sub && strcmp(sub, "CLEAR") == 0) {
            for (int i = 0; i < 5; i++) timerSlots[i].active = false;
            send_ack("TIMER_CLEAR");
        } else send_error("TIMER_FORMAT");
    }

    /* -------- STATUS -------- */
    else if (strcmp(mainTok, "STATUS") == 0) {
        UART_SendStatusPacket();
    }

    /* -------- UNKNOWN -------- */
    else {
        send_error("UNKNOWN_CMD");
    }
}
