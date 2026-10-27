#ifndef __CRC_H
#define __CRC_H

#include "main.h"

#include <stdint.h>

uint16_t Com_CRC16(uint8_t *buf, uint16_t len);
uint8_t Com_AddCheck(uint8_t *buf, uint16_t len);

#endif
