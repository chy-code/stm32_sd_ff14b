#include "stm32f10x.h"
#include "SD_SPI.h"


void SD_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;

    // 使能GPIOA, SPI1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    // 配置GPIO引脚 (SCK, MISO, MOSI)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置GPIO引脚 (CS)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_SetBits(GPIOA, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    // SPI1 配置
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master; // 主设备
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // 8bit
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;// 串行同步时钟的空闲状态为高电平
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;// 第二个跳变沿数据被采样
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // NSS信号由软件控制
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7; // CRC 项计算的多项式
    SPI_Init(SPI1, &SPI_InitStruct);

    SPI_Cmd(SPI1, ENABLE); // 使能SPI1
}


void SD_SPI_DeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    SPI_Cmd(SPI1, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    GPIO_SetBits(GPIOA, GPIO_Pin_4);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}


uint8_t SD_SPI_Exchange(uint8_t data)
{
    /* 等待发送缓冲器为空闲状态*/
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    SPI_I2S_SendData(SPI1, data);

    /* 等待接收缓冲器为非空状态*/
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    return SPI_I2S_ReceiveData(SPI1);
}


void SD_SPI_SetCSLow(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}


void SD_SPI_SetCSHigh(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
}


void SD_SPI_SetSpeedLow(void)
{
    uint16_t tmpreg = SPI1->CR1;
    tmpreg &= ~0x38;
    tmpreg |= SPI_BaudRatePrescaler_256;
    SPI1->CR1 = tmpreg;
}


void SD_SPI_SetSpeedHigh(void)
{
    uint16_t tmpreg = SPI1->CR1;
    tmpreg &= ~0x38;
    tmpreg |= SPI_BaudRatePrescaler_2;
    SPI1->CR1 = tmpreg;
}

