#ifndef _SD_UTIL_H
#define _SD_UTIL_H

#include <stdint.h>

uint8_t SDU_CalcCRC7(const uint8_t *buf, uint32_t count);
uint16_t SDU_CalcCRC16(const uint8_t *buf, uint32_t count);

#endif
