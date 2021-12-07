#ifndef _LED_H
#define _LED_H


#include <stdint.h>

// LED 类型定义
typedef enum {
    LED_RED,
    LED_GREEN
} LEDType;


void LED_Init(void);

void LED_On(LEDType type);
void LED_Off(LEDType type);
void LED_ToggleState(LEDType type);

#endif
