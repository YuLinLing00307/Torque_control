#include "dev_nj70.h"

void Dev_NJ70_Init(uint8_t *rx_buf, uint16_t len)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, len);
	__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);   /* 关掉半传输中断，只要整帧 */
}

void Dev_NJ70_Send_Quick_Cmd(uint8_t *tx_buffer)
{
	tx_buffer[0] = 0x01;    /* 节点号 */
	tx_buffer[1] = 0x06;    /* 功能码：快捷读传感器数据 */

	Bsp_rs485_tx(tx_buffer, 2);
}
