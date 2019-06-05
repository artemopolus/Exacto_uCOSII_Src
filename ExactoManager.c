#include "includes.h"
#include <stm32f10x.h>
#include "exacto_struct.h"
#include "exacto_defs.h"

extern OS_EVENT * pMailStm32;
extern OS_FLAG_GRP * pFlgSensors;
extern OS_EVENT * pEvSensorBuff;
extern OS_EVENT  * pSism330;

extern s8    cBuf[16];
extern const uint8_t CntExactoStm32States;
extern uint8_t * ExactoStm32States;

extern ExactoLBIdata ExactoBuffer;
extern INT16U BaseDelay;

extern ExactoSensorSet lsm303;
extern ExactoSensorSet bmp280;
extern ExactoSensorSet ism330;

extern void SendStr(s8* ptr); 
extern void SendStrFixLen(uint8_t * ptr, uint8_t cnt);
extern s8* Dec_Convert(s8* buf, s32 value);

uint8_t Exacto_getfrq_lsm303ah(void);
uint8_t Exacto_setfrq_lsm303ah(uint8_t mode);
uint8_t Exacto_slftst_lsm303ah(void);

uint8_t Exacto_getfrq_ism330_G(void);
uint8_t Exacto_setfrq_ism330(uint8_t mode);

uint8_t Exacto_getfrq_bmp280(void);
uint8_t Exacto_setfrq_bmp280(uint8_t mode);






#define CNTSTATES   10
                   //0123456789012345678901234567890
char StringMode[] = "STM32 FLAG MODE = xxx \n";
//                                1         2         3         4         5
                      //0123456789012345678901234567890123456789012345678901234567890
char StringSensors[] = "BD = xxxxxx LSM303 = xxx BMP280 = xxx ISM330 = xxx \n";
//                                 1         2         3         4         5
                      // 0123456789012345678901234567890123456789012345678901234567890
char StringSensInfo[] = "Sensor = xxxxxx Freq = xxx Delay = xxxxxx \n";

void ExactoStm32StatesChanged_Callback(uint8_t RegAdr, uint8_t RegVal, uint8_t * perr);
void ExactoStm32ReportStates_Callback(uint8_t RegAdr);

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
    if(!(src->cnt_lsm303 || src->cnt_bmp280 || src->cnt_ism330))   return 0;
    dst[0] = src->cnt_lsm303;
    dst[1] = src->cnt_bmp280;
    dst[2] = src->cnt_ism330;
    for(uint8_t i = 0; i < src->cnt_lsm303; i++) dst[i + 3]                                        = src->lsm303[i];
    for(uint8_t i = 0; i < src->cnt_bmp280; i++) dst[i + 3 + src->cnt_lsm303]                      = src->bmp280[i];
    for(uint8_t i = 0; i < src->cnt_ism330; i++) dst[i + 3 + src->cnt_lsm303 + src->cnt_bmp280]    = src->ism330[i];
		uint8_t allcount = (src->cnt_bmp280 + src->cnt_ism330 + src->cnt_lsm303 + 3);
    src->cnt_lsm303 = 0;
    src->cnt_bmp280 = 0;
    src->cnt_ism330 = 0;
    return allcount;
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

void SensorData2lsm303(SensorData * src)
{
	uint8_t i = 0, pData = ExactoBuffer.cnt_lsm303;
	for(i = 0; i < 6 ; i++) 	ExactoBuffer.lsm303[pData + i] = src->s1[i];
	for(i = 6; i < 12 ; i++) 	ExactoBuffer.lsm303[pData + i] = src->s1[i];
	ExactoBuffer.cnt_lsm303 += 12;
}
void SensorData2bmp280(SensorData * src)
{
	uint8_t i = 0, pData = ExactoBuffer.cnt_bmp280;
	for(i = 0; i < 6 ; i++) ExactoBuffer.bmp280[pData + i] = src->s1[i];
	ExactoBuffer.cnt_bmp280 += 6;
}
void SensorData2ism330(SensorData * src)
{
	uint8_t i = 0, pData = ExactoBuffer.cnt_ism330;
	for(i = 0; i < 14 ; i++) 	ExactoBuffer.lsm303[pData + i] = src->sL[i];
	ExactoBuffer.cnt_ism330 += 14;
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
                switch(msg->actiontype)
                {
                    case 2:
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
                    break;
                    case 1:
                    SendStr((int8_t*)"Read stm32 states:\n");
                    Dec_Convert((s8*)cBuf, msg->adr);
                    SendStr((int8_t*)"adr=");
                    SendStr((int8_t*)&cBuf[3]);
                    SendStr((int8_t*)"\n");
                    Dec_Convert((s8*)cBuf, ExactoStm32States[msg->adr]);
                    SendStr((int8_t*)"value=");
                    SendStr((int8_t*)&cBuf[3]);
                    SendStr((int8_t*)"\n");
                    
                    ExactoStm32ReportStates_Callback(msg->adr);
                    
                    break;
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
void ExactoStm32ReportStates_Callback(uint8_t RegAdr)
{
    uint8_t err;
    OS_FLAGS flags;
    uint8_t ctrl;
    switch (RegAdr)
    {
        case READSENSMODE_ES32A:
            flags = OSFlagQuery(pFlgSensors, &err);
            Dec_Convert((s8*)cBuf, flags);
            strcpy(&StringMode[18], (char*)&cBuf[3]);
            SendStr((int8_t*)StringMode);
            break;
        case SENDFREQ_ES32A:
            Dec_Convert((s8*)cBuf, BaseDelay);
            strcpy(&StringSensors[5], (char*)&cBuf[4]);
            ctrl = Exacto_getfrq_lsm303ah();
            Dec_Convert((s8*)cBuf, ctrl);
            strcpy(&StringSensors[21], (char*)&cBuf[7]);
				#ifdef BMP280
            ctrl = Exacto_getfrq_bmp280();
            Dec_Convert((s8*)cBuf, ctrl);
				#endif
            strcpy(&StringSensors[34], (char*)&cBuf[7]);
            ctrl = Exacto_getfrq_ism330_G();
            Dec_Convert((s8*)cBuf, ctrl);
            strcpy(&StringSensors[47], (char*)&cBuf[7]);
            SendStr((int8_t*)StringSensors);
            break;
        case SET_LSM303:
            strcpy(&StringSensInfo[9],lsm303.Name);
            ctrl = Exacto_getfrq_lsm303ah();
            Dec_Convert((s8*)cBuf, ctrl);
            strcpy(&StringSensInfo[23], (char*)&cBuf[7]);
            Dec_Convert((s8*)cBuf, lsm303.TDiscr);
            strcpy(&StringSensInfo[35], (char*)&cBuf[4]);
            break;
    }
}
void ExactoStm32StatesChanged_Callback(uint8_t RegAdr, uint8_t RegVal, uint8_t * perr)
{
		uint8_t flgerr = 0;
		OS_FLAGS tmp_flg;
		OS_CPU_SR cpu_sr = 0;
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
                                            &flgerr);
                    if(flgerr == OS_ERR_NONE) SendStr((int8_t*)"Switch to mode: waiting\n");
                    else
                    {
                        SendStr((int8_t*)"SWITCH_ERR:ALLWAITING_ESM\n");
                        uCOSFlagPost_Callback(&flgerr);
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
									
                    tmp_flg = OSFlagPost( pFlgSensors,
                                            FLG_LSM303 + FLG_BMP280 + FLG_ISM330,
                                            OS_FLAG_SET,
                                            perr);
									if(tmp_flg != 0x07)
										__NOP();
                    OSSemPost(pSism330);
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
            switch(RegVal)
            {
                case (EXACTO_FREQ_100HZ):
                    OS_ENTER_CRITICAL()
                    BaseDelay = OS_TIME_10mS;
                    OS_EXIT_CRITICAL()
                    break;
                case (EXACTO_FREQ_10HZ):
                    OS_ENTER_CRITICAL()
                    BaseDelay = OS_TIME_100mS;
                    OS_EXIT_CRITICAL()
                    break;
                case (EXACTO_FREQ_1HZ):
                    OS_ENTER_CRITICAL()
                    BaseDelay = OS_TICKS_PER_SEC;
                    OS_EXIT_CRITICAL()
                    break;
            }
          break;
        case SET_LSM303:
            //Set Frequency
            if(RegVal <= 3)  //0:100Hz 1:800Hz 2:1600Hz 3:6400Hz
            {
                if(Exacto_setfrq_lsm303ah(RegVal))  SendStr((int8_t*)"SET_FRQ_LSM303\n");
                else SendStr((int8_t*)"SET_ERR:lsm303\n");
            }
            //Selftest
            if(RegVal == 4)
            {
                if ( Exacto_slftst_lsm303ah() )
                    SendStr((int8_t*)"SLFTST: success\n");
                else
                    SendStr((int8_t*)"SLFTST: failed\n");
            }
            break;
    }
}
void        App_buffer(void * p_arg)
{
    SensorData * ValInput;
	uint8_t err;
    ExactoBuffer.cnt_lsm303 = 0;
    ExactoBuffer.cnt_bmp280 = 0;
    ExactoBuffer.cnt_ism330 = 0;
    while(DEF_TRUE)
    {
        ValInput = (SensorData*)OSQPend(pEvSensorBuff,0,&err);
        if(err == OS_ERR_NONE)
        {
            switch(ValInput->pSensor)
            {
                case FLG_LSM303:
                    //SetData2exactoLBIdata(ValInput->s1, ExactoBuffer.lsm303, &ExactoBuffer.cnt_lsm303);
										SensorData2lsm303(ValInput);
                    break;
                case FLG_BMP280:
                    //SetData2exactoLBIdata(ValInput->s1, ExactoBuffer.bmp280, &ExactoBuffer.cnt_bmp280);
										SensorData2bmp280(ValInput);
                    break;
                case FLG_ISM330:
                    //SetData2exactoLBIdata(ValInput->s1, ExactoBuffer.ism330, &ExactoBuffer.cnt_ism330);
										SensorData2ism330(ValInput);
                    break;
            }
            if((ExactoBuffer.cnt_lsm303       ==     (EXACTOLBIDATASIZE - 1))
                ||(ExactoBuffer.cnt_bmp280    ==     (EXACTOLBIDATASIZE - 1))
                ||(ExactoBuffer.cnt_ism330    ==     (EXACTOLBIDATASIZE - 1)))
            {
                ExactoLBIdataCLR(&ExactoBuffer);
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
