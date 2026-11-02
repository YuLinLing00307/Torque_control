#ifndef __COM_CRC_H
#define __COM_CRC_H

#include "main.h"
#include <stdint.h>

/* NJ70 力矩传感器：7 字节帧，前 6 字节的 ADD8 和放在第 7 字节 */
uint8_t Com_AddCheck(uint8_t *buf, uint16_t len);

#endif
