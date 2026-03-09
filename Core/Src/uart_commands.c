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

/* =========================================================
   TX HELPERS
========================================================= */

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

typedef struct
{
    uint8_t level;
    uint8_t motorStatus;
    char mode[12];

} StatusSnapshot;

static StatusSnapshot lastSent = {255,255,"INIT"};

void UART_InitCommandSystem(void)
{
    lastSent.level = 255;
    lastSent.motorStatus = 255;
    strcpy(lastSent.mode,"INIT");
}

/* =========================================================
   SAFE TOKENIZER
========================================================= */

static char* next_token(char **ctx)
{
    if (!ctx || !*ctx) return NULL;

    char *start = *ctx;

    char *colon = strchr(start, ':');

    if (colon)
    {
        *colon = '\0';
        *ctx = colon + 1;
    }
    else
    {
        *ctx = NULL;
    }

    return start;
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
        (strcmp(lastSent.mode,mode)!=0);

    if (!changed) return;

    lastSent.level = level;
    lastSent.motorStatus = motorStatus;

    strncpy(lastSent.mode,mode,sizeof(lastSent.mode)-1);
    lastSent.mode[sizeof(lastSent.mode)-1]='\0';

    char buf[120];

    snprintf(buf,sizeof(buf),
        "@STATUS:MOTOR:%s:LEVEL:%d:MODE:%s#",
        motorStatus ? "ON":"OFF",
        level,
        mode);

    UART_TransmitPacket(buf);
}

static void parse_settings(char *ctx)
{
    if (!ctx) return;

    uint32_t gap_time_s     = ModelHandle_GetGapTime();
    uint8_t  retry_count    = ModelHandle_GetRetryCount();
    uint16_t maxrun_min     = ModelHandle_GetMaxRunTime();
    uint16_t lowV           = ModelHandle_GetUnderVolt();
    uint16_t highV          = ModelHandle_GetOverVolt();
    int16_t  overL          = (int16_t)ModelHandle_GetOverloadLimit();
    int16_t  underL         = (int16_t)ModelHandle_GetUnderloadLimit();
    uint8_t  pwrRes         = ModelHandle_GetPowerRestoreMode();

    uint32_t dry_run_time_s = 60;   // testing gap default

    uint8_t dryRun_en=1,testing_en=1,maxRun_en=1;
    uint8_t lowV_en=1,highV_en=1,overL_en=1,underL_en=1;

    uint8_t buzzEnable=1,buzzFull=1,buzzEmpty=1;

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

            /* ========= COMPACT PROTOCOL ========= */

            if(!strcmp(key,"D"))
                gap_time_s = v * 60UL;

            else if(!strcmp(key,"RC"))
                retry_count = v;

            else if(!strcmp(key,"T"))
                dry_run_time_s = v * 60UL;

            else if(!strcmp(key,"M"))
                maxrun_min = v;

            else if(!strcmp(key,"LV"))
                lowV = v;

            else if(!strcmp(key,"HV"))
                highV = v;

            else if(!strcmp(key,"OL"))
                overL = v;

            else if(!strcmp(key,"UL"))
                underL = v;

            else if(!strcmp(key,"PR"))
                pwrRes = v;

            else if(!strcmp(key,"DE"))
                dryRun_en = v;

            else if(!strcmp(key,"TE"))
                testing_en = v;

            else if(!strcmp(key,"ME"))
                maxRun_en = v;

            else if(!strcmp(key,"LVE"))
                lowV_en = v;

            else if(!strcmp(key,"HVE"))
                highV_en = v;

            else if(!strcmp(key,"OLE"))
                overL_en = v;

            else if(!strcmp(key,"ULE"))
                underL_en = v;

            else if(!strcmp(key,"BZ"))
                buzzEnable = v;

            else if(!strcmp(key,"BF"))
                buzzFull = v;

            else if(!strcmp(key,"BE"))
                buzzEmpty = v;

            /* ========= LEGACY PROTOCOL ========= */

            else if(!strcmp(key,"dryRunGap"))
                gap_time_s = v * 60UL;

            else if(!strcmp(key,"retryCount"))
                retry_count = v;

            else if(!strcmp(key,"testingGap"))
                dry_run_time_s = v * 60UL;

            else if(!strcmp(key,"maxRun"))
                maxrun_min = v;

            else if(!strcmp(key,"lowVolt"))
                lowV = v;

            else if(!strcmp(key,"highVolt"))
                highV = v;

            else if(!strcmp(key,"overLoad"))
                overL = v;

            else if(!strcmp(key,"underLoad"))
                underL = v;

            else if(!strcmp(key,"powerRestore"))
                pwrRes = v;
        }

        pair = strtok_r(NULL,";",&saveptr);
    }

    /* APPLY SETTINGS */

    ModelHandle_SetUserSettings(
        gap_time_s,
        retry_count,
        lowV,
        highV,
        overL,
        underL,
        maxrun_min
    );

    ModelHandle_SetDryRunTime(dry_run_time_s);

    ModelHandle_SetPowerRestoreMode(pwrRes);

    ModelHandle_SetDryRun(dryRun_en);

    ModelHandle_SetOverLoad(overL_en);

    ModelHandle_SetOverUnderVolt(lowV_en || highV_en);

    if(!buzzEnable)
        ModelHandle_SetBuzzerSettings(0,0,0);
    else
        ModelHandle_SetBuzzerSettings(
            buzzEnable,
            buzzFull,
            buzzEmpty
        );

    ack("@SOK#");
}
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
    char *cmd=next_token(&ctx);

    if(!cmd) return;

    /* ===== PING ===== */

    if(!strcmp(cmd,"PING"))
    {
        ack("@PONG#");
        return;
    }

    /* ===== SETTINGS ===== */

    else if(!strcmp(cmd,"SET") || !strcmp(cmd,"SETTINGS"))
    {
        parse_settings(ctx);
    }

    /* ===== STATUS ===== */

    else if(!strcmp(cmd,"STATUS"))
    {
        UART_SendStatusPacket();
        return;
    }

    /* ===== OTHER COMMANDS (UNCHANGED) ===== */

    else if(!strcmp(cmd,"RESTART"))
    {
        char *sub = next_token(&ctx);

        if(sub && !strcmp(sub,"ON"))
        {
            ModelHandle_StartRestart();
            ack("@RESTART_ON#");
        }
        else if(sub && !strcmp(sub,"OFF"))
        {
            ModelHandle_StopRestart();
            ack("@RESTART_OFF#");
        }
        else err("@FORMAT#");
    }

    else if(!strcmp(cmd,"MANUAL"))
    {
        char *state=next_token(&ctx);

        if(!state){ err("@FORMAT#"); return;}

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
        else err("@FORMAT#");
    }

    g_screenUpdatePending = true;
}
