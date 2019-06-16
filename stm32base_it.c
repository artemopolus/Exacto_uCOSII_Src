#include "includes.h"
#include <stm32f10x.h>  

#include "exacto_commander.h"
#include "exacto_buffer.h"
//#include "lsm303ah_reglib.h"
//#include "bmp280_reglib.h"
//#include "ism330_reglib.h"

#include "exacto_struct.h"
#include "exacto_defs.h"

#include "exacto_i2c_slave.h"

void SendStr(s8* ptr);

extern OS_EVENT * pMailStm32;
extern OS_EVENT * pI2C;

#ifdef ENABLE_TIME_MEAS

extern uint32_t ClkCnt;

#endif
CmdToStm32 bi2cToStm32;

void Exacto_init_i2c_slave(void)
{
    ConfigureMode_i2c_dma_slave();
    Activate2work_i2c_dma_slave();
    Handle_i2c_dma_slave();
}
void SetData2transmit(uint8_t * pData, const uint32_t datalen)
{
	SetDT2W2TR_fixlen_i2c_dma_slave(pData,datalen);
}
uint8_t CheckTransmitBuffer(void)
{
	#ifdef ENABLE_I2C_PENDING
	uint8_t errParser;
	OSSemPend(pI2C,0,&errParser);
	switch(errParser)
	{
		case OS_ERR_NONE:
			Block_TransmitInit_i2c_dma_slave(0);
			return 1;
		case OS_ERR_TIMEOUT:
			break;
		case OS_ERR_PEND_ABORT:
			break;
		case OS_ERR_EVENT_TYPE:
			break;
		case OS_ERR_PEND_ISR:
			break;
		case OS_ERR_PEVENT_NULL:
			break;
		case OS_ERR_PEND_LOCKED:
			break;
	}
	return 0;
	#else
	Block_TransmitInit_i2c_dma_slave(0);
	return 1;
	#endif
}
void ReleaseTransmitBuffer(void)
{
	Block_TransmitInit_i2c_dma_slave(1);
}
void SetInitTransmitData(void)
{
	uint8_t value = 1;
	Block_TransmitInit_i2c_dma_slave(0);
	while(SetValue2W2TR_i2c_dma_slave(value++))
	{
	}
	Block_TransmitInit_i2c_dma_slave(1);
}
void DMA1_Channel7_IRQHandler(void)
{
    switch(DMA_Body_RX_IRQHandler())
    {
        case 1:
					//Transfer receive complete
					__NOP();
					if(GetReceiveBufferValue(0) == 0)
					{
						bi2cToStm32.actiontype = GetReceiveBufferValue(1);
						bi2cToStm32.adr = GetReceiveBufferValue(2);
						bi2cToStm32.val = GetReceiveBufferValue(3);
						__NOP();
						uint8_t Merr = OSMboxPost(pMailStm32, (void *)&bi2cToStm32);
						if(Merr == OS_ERR_NONE)
						{
								__NOP();
						}
						else
						{
								__NOP();
								SendStr((int8_t*)"MBOX_ERR:bi2cToStm32 post\n");
						}
					}
          break;
        case 0:
            //error
            break;
        case 2:
            //unknown
            break;
    }
}

void DMA1_Channel6_IRQHandler(void)
{
    switch(DMA_Body_TX_IRQHandler())
    {
        case 1:
					//Transfer transmit complete
					#ifdef ENABLE_TIME_MEAS
					ClkCnt = OSTimeGet();
					#endif
				#ifdef ENABLE_I2C_PENDING
					OSSemPost(pI2C);
				#endif
					__NOP();	
          break;
        case 0:
            //error
					__NOP();
            break;
        case 2:
					__NOP();
            //unknown
            break;
    }
}

void I2C1_EV_IRQHandler(void)
{
    switch(I2C_DMA_RXTX_EV_IRQHandler())
		{
			case 1:
				#ifdef ENABLE_TIME_MEAS
			OSTimeSet(0L);
			#endif
				//transmit
			__NOP();
				break;
			case 2:
				//receive
				__NOP();
				break;
			case 3:
				//transfer compelete
				__NOP();
				break;
			case 0:
				__NOP();
				break;
		}
}

void I2C1_ER_IRQHandler(void)
{
	if(I2C_DMA_Body_ER_IRQHandler())
	{
		__NOP();
	}
	else
		__NOP();
}
