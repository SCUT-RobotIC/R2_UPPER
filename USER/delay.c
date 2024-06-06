#include "delay.h"

void delay_us(uint32_t us) 
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 8000000 * us); 
    while (delay--); 
}

void delay_ms(uint32_t ms) 
{
    delay_us(ms*1000);
}

