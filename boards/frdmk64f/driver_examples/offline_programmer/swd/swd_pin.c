#include <stdint.h>
#include <stdbool.h>
#include "fsl_gpio.h"
#include "swd_pin.h"

#if (__CORTEX_M == 4)
#define PAout(n)   BITBAND_REG(PTA->PDOR, n)
#define PAin(n)    BITBAND_REG(PTA->PDIR, n)
  
#define PBout(n)   BITBAND_REG(PTB->PDOR, n)
#define PBin(n)    BITBAND_REG(PTB->PDIR, n)

#define PCout(n)   BITBAND_REG(PTC->PDOR, n)
#define PCin(n)    BITBAND_REG(PTC->PDIR, n)

#define PDout(n)   BITBAND_REG(PTD->PDOR, n)
#define PDin(n)    BITBAND_REG(PTD->PDIR, n)

#define PEout(n)   BITBAND_REG(PTE->PDOR, n)
#define PEin(n)    BITBAND_REG(PTE->PDIR, n)

#define PFout(n)   BITBAND_REG(PTF->PDOR, n)
#define PFin(n)    BITBAND_REG(PTF->PDIR, n)

#define PGout(n)   BITBAND_REG(PTG->PDOR, n)
#define PGin(n)    BITBAND_REG(PTG->PDIR, n)
#endif

static uint32_t swd_speed = 50;

void set_swd_speed(int val)
{
    swd_speed = val;
}

void DOUT(uint32_t val)
{
    PBout(3) = val;
}

void SW_PinInit(void)
{
    /* release trst pin */
    DDIR(1);
    DOUT(0);
    TCK(0);
    TRST(1);

}

void DDIR(uint32_t val)
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
    volatile int i;
    for(i=0; i<swd_speed; i++)
    {
        __NOP();
    }
}

void TCK(uint32_t val)
{
    PBout(2) = val;
}

void TRST(uint32_t val)
{
    PBout(10) = val;
}

uint32_t SW_READ_BIT(void)
{
    uint32_t bit;
    TCK(0);
    DELAY_US(1);
    bit = PBin(3);
    TCK(1);
    DELAY_US(1);
    return bit;
}

void SW_WRITE_BIT(uint32_t bit)
{
    DOUT(bit);
    TCK(0);
    DELAY_US(1);
    TCK(1);
    DELAY_US(1);
}

