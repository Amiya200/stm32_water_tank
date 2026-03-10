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
extern volatile uint8_t motorStatus;

/* ---------------- ACK / ERR ---------------- */

static inline void ack(const char *msg)
{
    UART_TransmitPacket(msg);
}

static inline void err(const char *msg)
{
    UART_TransmitPacket(msg);
}

/* ---------------- STATUS SNAPSHOT ---------------- */

typedef struct
{
    uint8_t level;
    uint8_t motor;
    char mode[12];

} StatusSnapshot;

static StatusSnapshot lastSent = {255,255,"INIT"};

/* ---------------- INIT ---------------- */

void UART_InitCommandSystem(void)
{
    lastSent.level = 255;
    lastSent.motor = 255;
    strcpy(lastSent.mode,"INIT");
}

/* ---------------- TOKEN PARSER ---------------- */

static char* next_token(char **ctx)
{
    if(!ctx || !*ctx) return NULL;

    char *start = *ctx;
    char *sep = strchr(start, ':');

    if(sep)
    {
        *sep = '\0';
        *ctx = sep + 1;
    }
    else
    {
        *ctx = NULL;
    }

    return start;
}

/* ---------------- STATUS PACKET ---------------- */

void UART_SendStatusPacket(void)
{
    uint8_t level = ModelHandle_GetTankLevelPercent();

    const char *mode="IDLE";

    if(ModelHandle_IsRestartActive()) mode="RESTART";
    else if(ModelHandle_IsManualActive()) mode="MANUAL";
    else if(ModelHandle_IsAutoActive()) mode="AUTO";
    else if(timerSlots[0].enabled||
            timerSlots[1].enabled||
            timerSlots[2].enabled||
            timerSlots[3].enabled||
            timerSlots[4].enabled)
        mode="TIMER";

    bool changed =
        (lastSent.level!=level) ||
        (lastSent.motor!=motorStatus) ||
        strcmp(lastSent.mode,mode);

    if(!changed) return;

    lastSent.level = level;
    lastSent.motor = motorStatus;

    strncpy(lastSent.mode,mode,sizeof(lastSent.mode)-1);

    char buf[80];

    snprintf(buf,sizeof(buf),
             "@STATUS:%s:%d:%s#",
             motorStatus?"ON":"OFF",
             level,
             mode);

    UART_TransmitPacket(buf);
}

/* ---------------- SETTINGS PARSER ---------------- */

static void parse_settings(char *ctx)
{
    if(!ctx) return;

    uint32_t gap_time_s = ModelHandle_GetGapTime();
    uint8_t retry = ModelHandle_GetRetryCount();
    uint16_t maxrun = ModelHandle_GetMaxRunTime();

    uint16_t lowV = ModelHandle_GetUnderVolt();
    uint16_t highV = ModelHandle_GetOverVolt();

    int16_t overL  = (int16_t)ModelHandle_GetOverloadLimit();
    int16_t underL = (int16_t)ModelHandle_GetUnderloadLimit();

    uint8_t powerRestore = ModelHandle_GetPowerRestoreMode();

    uint32_t dry_time = 60;

    char *saveptr;
    char *pair = strtok_r(ctx,";",&saveptr);

    while(pair)
    {
        char *eq = strchr(pair,'=');

        if(eq)
        {
            *eq='\0';

            char *key = pair;
            char *val = eq+1;

            int v = atoi(val);

            if(!strcmp(key,"D")) gap_time_s = v*60UL;
            else if(!strcmp(key,"RC")) retry=v;
            else if(!strcmp(key,"T")) dry_time=v*60UL;
            else if(!strcmp(key,"M")) maxrun=v;
            else if(!strcmp(key,"LV")) lowV=v;
            else if(!strcmp(key,"HV")) highV=v;
            else if(!strcmp(key,"OL")) overL=v;
            else if(!strcmp(key,"UL")) underL=v;
            else if(!strcmp(key,"PR")) powerRestore=v;
        }

        pair = strtok_r(NULL,";",&saveptr);
    }

    ModelHandle_SetUserSettings(
        gap_time_s,
        retry,
        lowV,
        highV,
        overL,
        underL,
        maxrun
    );

    ModelHandle_SetDryRunTime(dry_time);

    ModelHandle_SetPowerRestoreMode(powerRestore);

    ack("@SOK#");
}

/* ---------------- COMMAND HANDLER ---------------- */

void UART_HandleCommand(const char *pkt)
{
    if(!pkt || !*pkt) return;

    char buf[UART_RX_BUFFER_SIZE];

    strncpy(buf,pkt,sizeof(buf)-1);
    buf[sizeof(buf)-1]='\0';

    if(buf[0]=='@')
        memmove(buf,buf+1,strlen(buf));

    char *end=strchr(buf,'#');
    if(end) *end='\0';

    char *ctx=buf;

    char *cmd = next_token(&ctx);

    if(!cmd) return;

    /* -------- PING -------- */

    if(!strcmp(cmd,"PING"))
    {
        ack("@PONG#");
        return;
    }

    /* -------- STATUS -------- */

    if(!strcmp(cmd,"STATUS"))
    {
        UART_SendStatusPacket();
        return;
    }

    /* -------- SETTINGS -------- */

    if(!strcmp(cmd,"SET") || !strcmp(cmd,"SETTINGS"))
    {
        parse_settings(ctx);
        return;
    }

    /* -------- MANUAL -------- */

    if(!strcmp(cmd,"MANUAL"))
    {
        char *state=next_token(&ctx);

        if(!state){ err("@FORMAT#"); return; }

        if(!strcmp(state,"ON"))
        {
            if(!ModelHandle_IsManualActive())
                ModelHandle_ToggleManual();

            ack("@MANUAL_ON#");
        }
        else if(!strcmp(state,"OFF"))
        {
            if(ModelHandle_IsManualActive())
                ModelHandle_ToggleManual();

            ack("@MANUAL_OFF#");
        }

        return;
    }

    /* -------- AUTO -------- */

    if(!strcmp(cmd,"AUTO"))
    {
        char *state=next_token(&ctx);

        if(!state){ err("@FORMAT#"); return; }

        if(!strcmp(state,"ON"))
        {
            ModelHandle_StartAuto(
                ModelHandle_GetGapTime(),
                ModelHandle_GetMaxRunTime(),
                ModelHandle_GetRetryCount()
            );

            ack("@AUTO_ON#");
        }
        else if(!strcmp(state,"OFF"))
        {
            ModelHandle_StopAuto();

            ack("@AUTO_OFF#");
        }

        return;
    }
    /* -------- TIMER COMMAND -------- */

    if(!strcmp(cmd,"TIMER"))
    {
        char *sub = next_token(&ctx);

        if(!sub)
        {
            err("@FORMAT#");
            return;
        }

        /* -------- TIMER SET -------- */

        if(!strcmp(sub,"SET"))
        {
            char *slot  = next_token(&ctx);
            char *days  = next_token(&ctx);
            char *onH   = next_token(&ctx);
            char *onM   = next_token(&ctx);
            char *offH  = next_token(&ctx);
            char *offM  = next_token(&ctx);
            char *en    = next_token(&ctx);

            if(!slot || !days || !onH || !onM || !offH || !offM || !en)
            {
                err("@FORMAT#");
                return;
            }

            /* Convert slot from APP format (1-5) → firmware index (0-4) */

            int slotNum = atoi(slot);

            if(slotNum < 1 || slotNum > 5)
            {
                err("@SLOT_ERR#");
                return;
            }

            uint8_t s = slotNum - 1;

            uint8_t h1 = atoi(onH);
            uint8_t m1 = atoi(onM);
            uint8_t h2 = atoi(offH);
            uint8_t m2 = atoi(offM);
            uint8_t enabled = atoi(en);

            /* Convert day string to mask */

            uint8_t dayMask = 0;

            if(strstr(days,"mon")) dayMask |= (1<<0);
            if(strstr(days,"tue")) dayMask |= (1<<1);
            if(strstr(days,"wed")) dayMask |= (1<<2);
            if(strstr(days,"thu")) dayMask |= (1<<3);
            if(strstr(days,"fri")) dayMask |= (1<<4);
            if(strstr(days,"sat")) dayMask |= (1<<5);
            if(strstr(days,"sun")) dayMask |= (1<<6);

            /* Store slot */

            timerSlots[s].onHour    = h1;
            timerSlots[s].onMinute  = m1;
            timerSlots[s].offHour   = h2;
            timerSlots[s].offMinute = m2;
            timerSlots[s].dayMask   = dayMask;
            timerSlots[s].enabled   = enabled;

            /* Save to EEPROM */

            ModelHandle_SaveTimerToEEPROM();

            ack("@TIMER_SET_OK#");

            return;
        }

        /* -------- TIMER ON -------- */

        if(!strcmp(sub,"ON"))
        {
            timerActive = true;

            ModelHandle_StartTimerNearestSlot();

            ack("@TIMER_ON#");

            return;
        }

        /* -------- TIMER OFF -------- */

        if(!strcmp(sub,"OFF"))
        {
            timerActive = false;

            ModelHandle_StopTimer();

            ack("@TIMER_OFF#");

            return;
        }

        err("@FORMAT#");
    }
    if(!strcmp(cmd,"SEMIAUTO"))
    {
        char *state = next_token(&ctx);

        if(!state){ err("@FORMAT#"); return; }

        if(!strcmp(state,"ON"))
        {
            ModelHandle_StartSemiAuto();

            ack("@SEMIAUTO_ON#");
        }
        else if(!strcmp(state,"OFF"))
        {
            ModelHandle_StopSemiAuto();

            ack("@SEMIAUTO_OFF#");
        }

        return;
    }
    if(!strcmp(cmd,"COUNTDOWN"))
    {
        char *sec=next_token(&ctx);

        if(!sec){ err("@FORMAT#"); return; }

        uint32_t seconds = atoi(sec);

        ModelHandle_StartCountdown(seconds);

        ack("@COUNTDOWN_ON#");

        return;
    }

    /* -------- TWIST -------- */

    if(!strcmp(cmd,"TWIST"))
    {
        char *state=next_token(&ctx);

        if(!state){ err("@FORMAT#"); return; }

        if(!strcmp(state,"ON"))
        {
            ModelHandle_StartTwist(5,5,0,0,0,0);
            ack("@TWIST_ON#");
        }
        else if(!strcmp(state,"OFF"))
        {
            ModelHandle_StopTwist();
            ack("@TWIST_OFF#");
        }

        return;
    }

    /* -------- RESTART -------- */

    if(!strcmp(cmd,"RESTART"))
    {
        char *state=next_token(&ctx);

        if(!state){ err("@FORMAT#"); return; }

        if(!strcmp(state,"ON"))
        {
            ModelHandle_StartRestart();
            ack("@RESTART_ON#");
        }
        else if(!strcmp(state,"OFF"))
        {
            ModelHandle_StopRestart();
            ack("@RESTART_OFF#");
        }

        return;
    }

    err("@UNKNOWN#");

    g_screenUpdatePending = true;
}
