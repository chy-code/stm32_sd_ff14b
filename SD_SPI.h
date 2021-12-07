#ifndef _SD_SPI_H
#define _SD_SPI_H

#include <stdint.h>

void SD_SPI_Init(void);
void SD_SPI_DeInit(void);
void SD_SPI_SetCSLow(void);
void SD_SPI_SetCSHigh(void);
void SD_SPI_SetSpeedLow(void);
void SD_SPI_SetSpeedHigh(void);
uint8_t SD_SPI_Exchange(uint8_t data);

#endif
