#ifndef __SWD_PIN_H__
#define __SWD_PIN_H__

#include <stdint.h>

/* pin interface */
void SW_PinInit(void);
void DDIR(uint32_t val);
void TRST(uint32_t val);
void DOUT(uint32_t val);
void TCK(uint32_t val);
uint32_t SW_READ_BIT(void);
void SW_WRITE_BIT(uint32_t bit);
void PIN_DELAY(void);
void DELAY_US(uint32_t us);

#endif


