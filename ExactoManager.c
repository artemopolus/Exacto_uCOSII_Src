#include "includes.h"
#include <stm32f10x.h>
#include "exacto_struct.h"
#include "exacto_defs.h"

extern OS_EVENT * pMailStm32;
extern OS_FLAG_GRP * pFlgSensors;
extern void SendStr(s8* ptr); 
extern s8* Dec_Convert(s8* buf, s32 value);
extern s8    cBuf[16];
extern const uint8_t CntExactoStm32States;
extern uint8_t * ExactoStm32States;
extern OS_EVENT * pEvSensorBuff;
extern ExactoLBIdata buffer;

#define CNTSTATES   10

void ExactoStm32StatesChanged_Callback(uint8_t RegAdr, uint8_t RegVal, uint8_t * perr);

void uCOSFlagPost_Callback( uint8_t * perr)
{
    switch(*perr)
    {
        case OS_ERR_FLAG_INVALID_PGRP:
            SendStr((int8_t*)"FLGPSTERR:OS_ERR_FLAG_INVALID_PGRP\n");
            break;
        case OS_ERR_EVENT_TYPE:
            SendStr((int8_t*)"FLGPSTERR:OS_ERR_EVENT_TYPE\n");
            break;
        case OS_ERR_FLAG_INVALID_OPT:
            SendStr((int8_t*)"FLGPSTERR:OS_ERR_FLAG_INVALID_OPT\n");
            break;
    }
}

uint8_t ExactoLBIdataCLR(ExactoLBIdata * src)
{
    src->cnt_lsm303 = 0;
    src->cnt_bmp280 = 0;
    src->cnt_ism330 = 0;
    return 1;
}

uint8_t ExactoLBIdata2arrayUint8(ExactoLBIdata * src, uint8_t * dst)
{
    if(src->cnt_lsm303 && src->cnt_bmp280 && src->cnt_ism330)   return 0;
    dst[0] = src->cnt_lsm303;
    dst[1] = src->cnt_bmp280;
    dst[2] = src->cnt_ism330;
    for(uint8_t i = 0; i < src->cnt_lsm303; i++) dst[i + 3]                                        = src->lsm303[i];
    for(uint8_t i = 0; i < src->cnt_bmp280; i++) dst[i + 3 + src->cnt_lsm303]                      = src->bmp280[i];
    for(uint8_t i = 0; i < src->cnt_ism330; i++) dst[i + 3 + src->cnt_lsm303 + src->cnt_bmp280]    = src->ism330[i];
    src->cnt_lsm303 = 0;
    src->cnt_bmp280 = 0;
    src->cnt_ism330 = 0;
    return (src->cnt_bmp280 + src->cnt_ism330 + src->cnt_lsm303 + 3);
}

void SetData2exactoLBIdata(uint8_t * src, uint8_t * dst, uint8_t * ptr)
{
    for(uint8_t i = 0; i < 6; i++) dst[*ptr + i] = src[i] ;
    * ptr += 6;
    if(* ptr >= EXACTOLBIDATASIZE)
    {
        * ptr = 0;
    }
}


void App_stm32(void * p_arg)
{
    INT8U err;
	uint8_t ferr;
    CmdToStm32 *msg;
    while(DEF_TRUE)
    {
        msg = (CmdToStm32*)OSMboxPend(pMailStm32, 0, &err);    
        if (err == OS_ERR_NONE) {
            if(msg->adr < CntExactoStm32States)
            {
                if(msg->actiontype)
                {
                    SendStr((int8_t*)"Write stm32 states:\n");
                    Dec_Convert((s8*)cBuf, msg->adr);
                    SendStr((int8_t*)"adr=");
                    SendStr((int8_t*)&cBuf[3]);
                    SendStr((int8_t*)"\n");
                    Dec_Convert((s8*)cBuf, msg->val);
                    SendStr((int8_t*)"value=");
                    SendStr((int8_t*)&cBuf[3]);
                    SendStr((int8_t*)"\n");
                    ExactoStm32StatesChanged_Callback(msg->adr,msg->val,&ferr);
                }
                else
                {
                    SendStr((int8_t*)"Read stm32 states:\n");
                    Dec_Convert((s8*)cBuf, msg->adr);
                    SendStr((int8_t*)"adr=");
                    SendStr((int8_t*)&cBuf[3]);
                    SendStr((int8_t*)"\n");
                    Dec_Convert((s8*)cBuf, ExactoStm32States[msg->adr]);
                    SendStr((int8_t*)"value=");
                    SendStr((int8_t*)&cBuf[3]);
                    SendStr((int8_t*)"\n");
                }
            }
        } else {
            switch(err)
			{
				case OS_ERR_TIMEOUT:
                    SendStr((int8_t*)"AP_STM32:OS_ERR_TIMEOUT\n");
					break;
				case OS_ERR_PEND_ABORT:
                    SendStr((int8_t*)"AP_STM32:OS_ERR_PEND_ABORT\n");
					break;
                case OS_ERR_EVENT_TYPE:
                    SendStr((int8_t*)"AP_STM32:OS_ERR_EVENT_TYPE\n");
					break;
                case OS_ERR_PEND_ISR:
                    SendStr((int8_t*)"AP_STM32:OS_ERR_PEND_ISR\n");
					break;
                case OS_ERR_PEVENT_NULL:
                    SendStr((int8_t*)"AP_STM32:OS_ERR_PEVENT_NULL\n");
					break;
                case OS_ERR_PEND_LOCKED:
                    SendStr((int8_t*)"AP_STM32:OS_ERR_PEND_LOCKED\n");
					break;
			}
        }
    }
}
void ExactoStm32StatesChanged_Callback(uint8_t RegAdr, uint8_t RegVal, uint8_t * perr)
{
    if(RegAdr >= CntExactoStm32States)
        return;
    ExactoStm32States[RegAdr] = RegVal;
    switch (RegAdr)
    {
        case READSENSMODE_ES32A:
            switch(RegVal)
            {
                case ALLWAITING_ESM:
                    OSFlagPost( pFlgSensors,
                                            FLG_LSM303 | FLG_BMP280 | FLG_ISM330,
                                            OS_FLAG_CLR,
                                            perr);
                    if(*perr == OS_ERR_NONE) SendStr((int8_t*)"Switch to mode: waiting\n");
                    else
                    {
                        SendStr((int8_t*)"SWITCH_ERR:ALLWAITING_ESM\n");
                        uCOSFlagPost_Callback(perr);
                    }
                    break;
                case ONLYLSM303_ESM:
                    OSFlagPost( pFlgSensors,
                                            FLG_LSM303,
                                            OS_FLAG_SET,
                                            perr);
                    if(*perr == OS_ERR_NONE) 
											SendStr((int8_t*)"Switch to mode: only lsm303\n");
                    else
                    {
                        SendStr((int8_t*)"SWITCH_ERR:ONLYLSM303_ESM\n");
                        uCOSFlagPost_Callback(perr);
                    }
                    break;
                case ALLRUNNING_ESM:
                    OSFlagPost( pFlgSensors,
                                            FLG_LSM303 | FLG_BMP280 | FLG_ISM330,
                                            OS_FLAG_SET,
                                            perr);
                    if(*perr == OS_ERR_NONE) SendStr((int8_t*)"Switch to mode: all sensors\n");
                    else
                    {
                        SendStr((int8_t*)"SWITCH_ERR:ALLRUNNING_ESM\n");
                        uCOSFlagPost_Callback(perr);
                    }
                    break;
            }
            break;
        case SENDFREQ_ES32A:
            break;
    }
}
void        App_buffer(void * p_arg)
{
    SensorData * ValInput;
	uint8_t err;
    buffer.cnt_lsm303 = 0;
    buffer.cnt_bmp280 = 0;
    buffer.cnt_ism330 = 0;
    while(DEF_TRUE)
    {
        ValInput = (SensorData*)OSQPend(pEvSensorBuff,0,&err);
        if(err == OS_ERR_NONE)
        {
            switch(ValInput->pSensor)
            {
                case FLG_LSM303:
                    SetData2exactoLBIdata(ValInput->s1, buffer.lsm303, &buffer.cnt_lsm303);
                    break;
                case FLG_BMP280:
                    SetData2exactoLBIdata(ValInput->s1, buffer.bmp280, &buffer.cnt_bmp280);
                    break;
                case FLG_ISM330:
                    SetData2exactoLBIdata(ValInput->s1, buffer.ism330, &buffer.cnt_ism330);
                    break;
            }
            if((buffer.cnt_lsm303       ==     (EXACTOLBIDATASIZE - 1))
                ||(buffer.cnt_bmp280    ==     (EXACTOLBIDATASIZE - 1))
                ||(buffer.cnt_ism330    ==     (EXACTOLBIDATASIZE - 1)))
            {
                ExactoLBIdataCLR(&buffer);
                SendStr((int8_t*)"AP_BUFF:FORCE_BUFFER_CLR\n");
            }
        }
        else
        {
            switch(err)
			{
				case OS_ERR_TIMEOUT:
                    SendStr((int8_t*)"AP_BUFF:OS_ERR_TIMEOUT\n");
					break;
				case OS_ERR_PEND_ABORT:
                    SendStr((int8_t*)"AP_BUFF:OS_ERR_PEND_ABORT\n");
					break;
                case OS_ERR_EVENT_TYPE:
                    SendStr((int8_t*)"AP_BUFF:OS_ERR_EVENT_TYPE\n");
					break;
                case OS_ERR_PEVENT_NULL:
                    SendStr((int8_t*)"AP_BUFF:OS_ERR_PEVENT_NULL\n");
					break;
                case OS_ERR_PEND_ISR:
                    SendStr((int8_t*)"AP_BUFF:OS_ERR_PEND_ISR\n");
					break;
                case OS_ERR_PEND_LOCKED:
                    SendStr((int8_t*)"AP_BUFF:OS_ERR_PEND_LOCKED\n");
					break;
			}
        }
//        if(ExactoLBIdataCLR(&buffer))
//        {
//            __NOP();
//        }
    }
}
