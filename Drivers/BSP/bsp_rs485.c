#include "bsp_rs485.h"

void Bsp_rs485_tx(uint8_t *buf, uint8_t len)
{
	RS485_RE(1);                                    /* 切到发送 */
	HAL_UART_Transmit(&huart2, buf, len, 1000);     /* 阻塞发送，115200 下 7 字节约 0.6 ms */
	RS485_RE(0);                                    /* 切回接收 */
}
