#ifndef __FLASH_KINETIS_H__
#define __FLASH_KINETIS_H__

#include <stdint.h>
#include "swd.h"

/* wdog type selection */
#define WDOG_TYPE_32        (3)
#define WDOG_TYPE_16        (2)
#define WDOG_TYPE_8         (1)
#define WDOG_TYPE_COP       (4)

/* program command selection */
#define PGM4                (0x06)          /* program phase program 4 byte */
#define PGM8                (0x07)          /* program phase program 8 byte */


typedef struct
{
    uint32_t init;
    uint32_t uninit;
    uint32_t erase_chip;
    uint32_t erase_sector;
    uint32_t program_page;
    FLASH_SYSCALL sys_call_param;
    uint32_t program_buffer;
    uint32_t algo_start;
    uint32_t algo_size;
    const uint32_t * image;
    uint32_t ram_to_flash_bytes_to_be_written;

} target_flash_t;

uint8_t target_flash_unlock_sequence(void);
uint8_t target_flash_init(const target_flash_t* flash, uint32_t wdog_base_addr, uint32_t wdog_type, uint32_t sector_size, uint32_t pgm_cmd);
uint8_t target_flash_program_page(const target_flash_t* flash, uint32_t addr, uint8_t * buf, uint32_t size);
uint8_t target_flash_erase_sector(const target_flash_t* flash, unsigned int addr);
uint8_t target_flash_erase_chip(const target_flash_t* flash);

#endif

