#include "com_crc.h"

/**
 * @brief Modbus CRC16计算
 * @param buf:待校验数据指针
 * @param len:数据长度
 * @retval 计算得到的CRC16结果
 */
uint16_t Com_CRC16(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i,j;
    for(i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint8_t Com_AddCheck(uint8_t *buf, uint16_t len)
{
	uint8_t sum = 0;
	for(uint16_t i=0;i<len;i++)
	{
		sum += buf[i];
	}
	return sum;
}

