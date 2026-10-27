#include "bsp_rs485.h"

void BSP_rs485_tx(uint8_t *buf, uint8_t len)
{
	RS485_RE(1); /* 进入发送模式 */
	HAL_UART_Transmit(&huart2,buf,len,1000); /* 串口 2 发送数据 */
	RS485_RE(0); /* 进入接收模式 */
}

