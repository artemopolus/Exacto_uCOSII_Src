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

extern OS_EVENT * pMailStm32;
#ifdef ENABLE_TIME_MEAS

extern uint32_t ClkCnt;

#endif


void Exacto_init_i2c_slave(void)
{
    ConfigureMode_i2c_dma_slave();
    Activate2work_i2c_dma_slave();
    Handle_i2c_dma_slave();
}
uint8_t SetNewValue2transmit(const uint8_t value)
{
	return SetValue2W2TR_i2c_dma_slave(value);
}
void SetInitTransmitData(void)
{
	uint8_t value = 1;
	while(SetValue2W2TR_i2c_dma_slave(value++))
	{
	}
}
void DMA1_Channel7_IRQHandler(void)
{
    switch(DMA_Body_RX_IRQHandler())
    {
        case 1:
					//Transfer receive complete
				__NOP();
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
					
				#ifdef ENABLE_TIME_MEAS
				ClkCnt = OSTimeGet();
				#endif
				__NOP();
				//Transfer complete
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
}
