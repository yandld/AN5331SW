#include <stdint.h>
#include <stdbool.h>

#include "swd_pin.h"


void swd_delay_us(uint32_t us)
{
    volatile int i;
    
    for(i=0; i< us; i++)
    {
        DELAY_US(1);
    }
}

static uint32_t swd_speed = 50;

void set_swd_speed(int val)
{
    swd_speed = val;
}



void SW_PinInit(void)
{
    /* release trst pin */
    SW_DIR(1);
    TDOUT = 0;
    TCLK = 0;
    TRST = 1;

}

void SW_DIR(uint32_t val)
{
    if(val)
    {
        GPIOB->PDDR |= (1<<3);
    }
    else
    {
        GPIOB->PDDR &= ~(1<<3);
    }
    
}


void DELAY_US(uint32_t us)
{
    volatile int i, j;
    for(i=0; i<us; i++)
    {
        for(j=0; j<swd_speed; j++)
        {
            __NOP();
            
        }
    }
}



void swd_hw_write_trst(uint8_t val)
{
    TRST = val;
}




uint32_t SW_READ_BIT(void)
{
    uint32_t bit;
    TCLK = 0;
    DELAY_US(1);
    bit = PBin(3);
    TCLK = 1;
    DELAY_US(1);
    return bit;
}

void SW_WRITE_BIT(uint32_t bit)
{
    TDOUT = bit;
    TCLK = 0;
    DELAY_US(1);
    TCLK = 1;
    DELAY_US(1);
}

