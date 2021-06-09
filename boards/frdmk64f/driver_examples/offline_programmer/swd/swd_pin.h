#ifndef __SWD_PIN_H__
#define __SWD_PIN_H__

#include <stdint.h>
#include "fsl_gpio.h"

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


#define TRST    PBout(10)
#define TCLK    PBout(2)
#define TDOUT   PBout(3)
#define TDIN    PBin(3)

/* pin interface */
void SW_PinInit(void);
void SW_DIR(uint32_t val);
uint32_t SW_READ_BIT(void);
void swd_delay_us(uint32_t us);
void SW_WRITE_BIT(uint32_t bit);
void PIN_DELAY(void);
void DELAY_US(uint32_t us);
void swd_hw_write_trst(uint8_t val);
uint8_t swd_hw_read_trst(void);

#endif


