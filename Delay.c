#include "stm32f10x.h"
#include "Delay.h"


static const uint32_t MaxLoad = 0xFFFFFF;


__STATIC_FORCEINLINE void DelayTicks(uint32_t ticks)
{
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

    /* Disable SysTick counter */
    SysTick->CTRL = 0;
}


__STATIC_FORCEINLINE void DelayLongTicks(uint64_t ticks)
{
    do {
        if (ticks > MaxLoad) {
            DelayTicks(MaxLoad);
            ticks -= MaxLoad;
        }
        else {
            DelayTicks((uint32_t)ticks);
            break;
        }
    } while (ticks > 0);
}


void DelayMs(uint32_t msec)
{
    DelayLongTicks(SystemCoreClock / 1000 * msec);
}


void DelayUs(uint32_t usec)
{
    DelayLongTicks(SystemCoreClock / 1000000 * usec);
}
