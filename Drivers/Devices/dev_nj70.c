#include "dev_nj70.h"

void Dev_NJ70_Send_Quick_Cmd(uint8_t* tx_buffer)
{
	tx_buffer[0] = 0x01;
	tx_buffer[1] = 0x06;
	
	BSP_rs485_tx(tx_buffer,2);
}
