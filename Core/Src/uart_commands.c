#include "uart_commands.h"
#include "uart.h"
#include "model_handle.h"
#include "relay.h"
#include "rtc_i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static inline void ack(const char *msg)
{
    char small[40];
    snprintf(small, sizeof(small), "ACK:%s", msg);
    UART_TransmitPacket(small);
}
static inline void err(const char *msg)
{
    char small[40];
    snprintf(small, sizeof(small), "ERR:%s", msg);
    UART_TransmitPacket(small);
}
void UART_SendStatusPacket(void)
{
    extern ADC_Data adcData;
    int submerged = 0;
    for (int i = 0; i < 5; i++) {
        if (adcData.voltages[i] < 0.1f) submerged++;
    }

    const char *motor = motorStatus ? "ON" : "OFF";
    char buf[48];
    snprintf(buf, sizeof(buf), "STATUS:MOTOR:%s:LEVEL:%d", motor, submerged);
    UART_TransmitPacket(buf);
}

void UART_HandleCommand(const char *pkt)
{
    if (!pkt || pkt[0] == '\0') return;

    char cmd[UART_RX_BUFFER_SIZE];
    strncpy(cmd, pkt, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';

    if (cmd[0] == '@') memmove(cmd, cmd + 1, strlen(cmd));
    char *end = strchr(cmd, '#');
    if (end) *end = '\0';

    char *t = strtok(cmd, ":");
    if (!t) return;

    if (!strcmp(t, "PING")) { ack("PONG"); return; }

    if (!strcmp(t, "MANUAL")) {
        char *s = strtok(NULL, ":");
        if (!s) { err("PARAM"); return; }
        if (!strcmp(s, "ON"))  { ModelHandle_ToggleManual(); ack("MANUAL_ON"); }
        else if (!strcmp(s, "OFF")) { ModelHandle_StopAllModesAndMotor(); ack("MANUAL_OFF"); }
        else err("FORMAT");
        return;
    }

    if (!strcmp(t, "TWIST")) {
        char *sub = strtok(NULL, ":");
        if (sub && !strcmp(sub, "SET")) {
            uint16_t on = atoi(strtok(NULL, ":"));
            uint16_t off = atoi(strtok(NULL, ":"));
            ModelHandle_StartTwist(on, off);
            ack("TWIST_SET");
        } else if (sub && !strcmp(sub, "STOP")) {
            ModelHandle_StopTwist();
            ack("TWIST_STOP");
        }
        return;
    }

    if (!strcmp(t, "COUNTDOWN")) {
        char *s = strtok(NULL, ":");
        if (s && !strcmp(s, "ON")) {
            uint16_t min = atoi(strtok(NULL, ":"));
            if (min == 0) min = 1;
            ModelHandle_StartCountdown(min * 60, 1);
            ack("COUNTDOWN_ON");
        } else if (s && !strcmp(s, "OFF")) {
            ModelHandle_StopCountdown();
            ack("COUNTDOWN_OFF");
        }
        return;
    }

    if (!strcmp(t, "STATUS")) {
        UART_SendStatusPacket();
        return;
    }

    err("UNKNOWN");
}
