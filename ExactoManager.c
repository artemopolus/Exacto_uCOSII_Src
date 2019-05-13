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

#define CNTSTATES   10

void ExactoStm32StatesChanged_Callback(uint8_t RegAdr, uint8_t RegVal, uint8_t * perr);


void App_stm32(void * p_arg)
{
    INT8U err;
		uint8_t ferr;
    CmdToStm32 *msg;
    while(DEF_TRUE)
    {
        msg = (CmdToStm32*)OSMboxPend(pMailStm32, 0, &err);
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
        if (err == OS_ERR_NONE) {
        /* Code for received message */
        } else {
        /* Code for message not received within timeout */
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
							break;
						case ONLYLSM303_ESM:
							OSFlagPost( pFlgSensors,
													FLG_LSM303,
													OS_FLAG_SET,
													perr);
							break;
						case ALLRUNNING_ESM:
							OSFlagPost( pFlgSensors,
													FLG_LSM303 | FLG_BMP280 | FLG_ISM330,
													OS_FLAG_SET,
													perr);
							break;
					}
            break;
        case SENDFREQ_ES32A:
            break;
    }
}
