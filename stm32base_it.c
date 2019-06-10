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


void Exacto_init_i2c_slave(void)
{
    ConfigureMode_i2c_dma_slave();
    Activate2work_i2c_dma_slave();
    Handle_i2c_dma_slave();
}
void Transfer_I2C_Handler(void)
{
    switch(Transfer_Complete_i2c_dma_slave())
    {
        case 0:
            break;
        case 1:
            //Sending complete
            break;
        case 2:
            //Getting complete
            break;
    }
}
void DMA1_Channel7_IRQHandler(void)
{
    switch(DMA_Body_RX_IRQHandler())
    {
        case 1:
					//Transfer complete
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
