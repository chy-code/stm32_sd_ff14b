#include "SD_Util.h"


uint8_t SDU_CalcCRC7(const uint8_t *buf, uint32_t count)
{
    const uint8_t POLYNOMIAL = 0x89;	// X^7 + X^3 + 1
    uint32_t i, j;
    uint8_t crc = 0;

    for (i = 0; i < count; i++) {
        crc ^= buf[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ (POLYNOMIAL << 1);
            else
                crc <<= 1;
        }
    }

    return crc >> 1;
}


uint16_t SDU_CalcCRC16(const uint8_t *buf, uint32_t count)
{
    uint32_t i;
    uint16_t crc = 0;
    for (i = 0; i < count; i++) {
        crc  = (crc >> 8) | (crc << 8);
        crc ^=  buf[i];
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }

    return crc;
}
