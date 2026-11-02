#include "com_crc.h"

/* ADD8 校验：逐字节累加，取低 8 位 */
uint8_t Com_AddCheck(uint8_t *buf, uint16_t len)
{
	uint8_t  sum = 0;

	for (uint16_t i = 0; i < len; i++)
	{
		sum += buf[i];
	}
	return sum;
}
