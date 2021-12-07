#include "stm32f10x.h"
#include "LED.h"

#define RED_LED_PORT	GPIOB
#define RED_LED_PIN		GPIO_Pin_5

#define GREEN_LED_PORT	GPIOE
#define GREEN_LED_PIN	GPIO_Pin_5


void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB
                           | RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin = RED_LED_PIN;
    GPIO_Init(RED_LED_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GREEN_LED_PIN;
    GPIO_Init(GREEN_LED_PORT, &GPIO_InitStruct);
}


void LED_On(LEDType which)
{
    switch (which) {
    case LED_RED:
        GPIO_ResetBits(RED_LED_PORT, RED_LED_PIN);
        break;
    case LED_GREEN:
        GPIO_ResetBits(GREEN_LED_PORT, GREEN_LED_PIN);
        break;
    }
}


void LED_Off(LEDType which)
{
    switch (which) {
    case LED_RED:
        GPIO_SetBits(RED_LED_PORT, RED_LED_PIN);
        break;
    case LED_GREEN:
        GPIO_SetBits(GREEN_LED_PORT, GREEN_LED_PIN);
        break;
    }
}


void LED_ToggleState(LEDType which)
{
    switch (which) {
    case LED_RED:
        RED_LED_PORT->ODR ^= RED_LED_PIN;
        break;
    case LED_GREEN:
        GREEN_LED_PORT->ODR ^= GREEN_LED_PIN;
        break;
    }
}

