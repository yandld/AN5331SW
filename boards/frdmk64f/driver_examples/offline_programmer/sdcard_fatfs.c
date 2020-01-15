/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fsl_uart.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_ftm.h"
#include "fsl_debug_console.h"
#include "ff.h"
#include "diskio.h"
#include "board.h"

#include "fsl_mpu.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "swd.h"
#include "swd_pin.h"
#include "flash_kinetis.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct
{
    const char* name;
    uint32_t wodg_base;
    uint32_t wdog_type;
    uint32_t sector_size;
    uint32_t pgm;
}AlgorithmParams_t;


AlgorithmParams_t AlgorithmTbl[] = 
{
    /*                                   WDOG_BASE    WDOG_TYPE     SEC_SIZE  PGM CMD */
    {"xxDN256/512",                 0x40052000u,    WDOG_TYPE_16,    2048,   PGM4},
    {"K60DN512",                    0x40052000u,    WDOG_TYPE_16,    2048,   PGM4},
    {"KL28",                        0x40076000u,    WDOG_TYPE_32,    2048,   PGM4},
    {"MK20DX128",                   0x40052000u,    WDOG_TYPE_16,    1024,   PGM4},
    {"KE15Z256",                    0x40052000u,    WDOG_TYPE_32,    2048,   PGM8},
    {"KL0/1/26/27",                 0x40047000u,    WDOG_TYPE_COP,   1024,   PGM4},
    {"xxFN1M",                      0x40052000u,    WDOG_TYPE_16,    4096,   PGM8},
    {"xxFN256",                     0x40052000u,    WDOG_TYPE_16,    4096,   PGM4},
};

/* buffer size (in byte) for read/write operations */
extern const target_flash_t flash;
#define TARGET_PFLASH_IAMGE_PATH        "/PIMAGE.BIN"
#define TARGET_DFLASH_IAMGE_PATH        "/DIMAGE.BIN"
#define MAX_SECTOR_SIZE                 (4096)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Delay some time.
 *
 * @param milliseconds Time united in milliseconds.
 */
void delay(uint32_t milliseconds);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static FATFS g_fileSystem; /* File system object */
static FIL f_pimage;   /* pflash image file */
static FIL f_dimage;   /* dflash image file */
static FIL f_config;   /* config file */
static uint8_t ImageBuf[MAX_SECTOR_SIZE];   /* Read buffer, maxed sector size on Kinetis*/
static AlgorithmParams_t Algorithm;
static bool IsDFlashImageExist = false;
static uint32_t FlashOption = 0;
/*******************************************************************************
 * Code
 ******************************************************************************/
 
void ERROR_TRACE(const char *log)
{
    printf("system halt!!!\r\n");
    printf("%s\r\n", log);
    LED_GREEN_OFF();
    LED_BLUE_OFF();
    LED_RED_ON();
    __disable_irq();
    while(1);
}
 
/* Delay some time united in milliseconds. */
void delay(uint32_t milliseconds)
{
    uint32_t i, j;

    for (i = 0; i < milliseconds; i++)
    {
        for (j = 0; j < 20000U; j++)
        {
            __asm("NOP");
        }
    }
}

void SWD_PinInit(void)
{
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_PortE);
    
    /* TRST */
    PORTB->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOB->PDDR |= (1<<10);
    
    /* SWCLK */
    PORTB->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK | PORT_PCR_SRE_MASK;
    GPIOB->PDDR |= (1<<2);
    
    /* SWDIO */
    PORTB->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOB->PDDR |= (1<<3);
    
    SW_PinInit();
    
    TRST(0);
    SWJ_InitDebug();
    
    uint32_t id;
    SWJ_ReadDP(DP_IDCODE, &id);
    printf("DP-IDR:0x%X\r\n", id);
    
    SWJ_ReadAP(0x000000FC, &id);
    printf("AHB_AP_IDR:0x%X\r\n", id);

    SWJ_ReadAP(0x010000FC, &id);
    printf("MDM-AP ID:0x%X\r\n", id);
}

void SWD_PinDeInit(void)
{
    GPIOB->PDDR &= ~(1<<10);
    
    /* SWCLK */
    GPIOB->PDDR &= ~(1<<2);
    
    /* SWDIO */
    GPIOB->PDDR &= ~(1<<3);
}

/*!
 * @brief Main function
 */
int main(void)
{
    FRESULT error;
    uint32_t val, i;
    int ret;
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    MPU_Enable(MPU, false);
    LED_RED_INIT(1);
    LED_GREEN_INIT(1);
    LED_BLUE_INIT(1);
    

    /* output 3Hz wave for disable external WDOG reset signal*/
    CLOCK_EnableClock(kCLOCK_PortC);
    PORTC->PCR[10] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOC->PDDR |= (1<<10);
    pit_config_t pit_config;
    pit_config.enableRunInDebug = false;
    PIT_Init(PIT, &pit_config);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, 1000*1000*10);
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
    PIT_StartTimer(PIT, kPIT_Chnl_0);
    NVIC_EnableIRQ(PIT0_IRQn);
    
    PRINTF("FRDM K64 offline programmer.\r\n");
    PRINTF("init SD card...\r\n");
    
    delay(100U);

    if (f_mount(&g_fileSystem, driverNumberBuffer, 0U))
    {
        PRINTF("Mount volume failed.\r\n");
        return -1;
    }
    
#if (_FS_RPATH >= 2U)
    error = f_chdrive((char const *)&driverNumberBuffer[0U]);
    if (error)
    {
        PRINTF("Change drive failed.\r\n");
        return -1;
    }
#endif

    PRINTF("SD card init OK\r\n");
    
    printf("Sselect target:\r\n");
    for(i=0; i<ARRAY_SIZE(AlgorithmTbl); i++)
    {
        printf("%d: %-16s WDOG:%d SEC_SIZE:%d %s\r\n", i, AlgorithmTbl[i].name, AlgorithmTbl[i].wdog_type, AlgorithmTbl[i].sector_size, (AlgorithmTbl[i].pgm == 6)?("PGM4"):("PGM8"));
    }
    
    printf("init swd...\r\n");
    /* init swd */
    SWD_PinInit();
    
    /* reading configuraton */
    UINT cbr;
    error = f_open(&f_config, "/setting.ini", FA_READ);
    if(error != FR_OK)
    {
        printf("Cannot find %s!\r\n", "/setting.ini");
        LED_BLUE_ON();
        while(1);
    }
    else
    {
        f_read(&f_config, ImageBuf, f_config.fsize, &cbr);
        printf("\r\nCONFIGURATION:\r\n%s\r\n", ImageBuf);
        char *p = strstr((const char*)ImageBuf, "FLASH_OPTION=");
        if(p)
        {
            FlashOption = strtoul(p+strlen("FLASH_OPTION="), 0, 0);
        }

        printf("FlashOption:%d\r\n", FlashOption);
        Algorithm = AlgorithmTbl[FlashOption];
        printf("\r\n%s WdogType:%d WdogBaseAddr:0x%08X, SectorSize:%d  FlashCmd:%d\r\n", Algorithm.name, Algorithm.wdog_type, Algorithm.wodg_base, Algorithm.sector_size, Algorithm.pgm);
    }
    
    /* check and open program flash image */
    error = f_open(&f_pimage, TARGET_PFLASH_IAMGE_PATH, FA_READ);
    if(error != FR_OK)
    {
        printf("Cannot find %s!\r\n", TARGET_PFLASH_IAMGE_PATH);
        LED_BLUE_ON();
        while(1);
    }
    else
    {
        PRINTF("pflash image find %s, size:%d\r\n", TARGET_PFLASH_IAMGE_PATH, f_pimage.fsize);
    }
    
    /* check and open data flash image */
    error = f_open(&f_dimage, TARGET_DFLASH_IAMGE_PATH, FA_READ);
    if(error != FR_OK)
    {
        printf("Cannot find %s!\r\n", TARGET_DFLASH_IAMGE_PATH);
    }
    else
    {
        PRINTF("dflash image find %s, size:%d\r\n", TARGET_DFLASH_IAMGE_PATH, f_dimage.fsize);
        IsDFlashImageExist = true;
    }
    
    delay(500);
    
    /* unlock Kinetis */
    target_flash_unlock_sequence();
    
    /* prepare download */
    SWJ_SetTargetState(RESET_PROGRAM);
    
    /* inject flash algorithm */
    ret = target_flash_init(&flash,  Algorithm.wodg_base, Algorithm.wdog_type, Algorithm.sector_size, Algorithm.pgm);
    if(ret)
    {
        ERROR_TRACE("dowloading flash algorithm failed");
    }
    
    uint32_t len;
    UINT br;
    uint32_t offset;
    
    /* erase program flash sector */
    offset = 0;
    len = f_pimage.fsize;
    
    while(offset < (len + Algorithm.sector_size))
    {
        printf("erase pflash addr:0x%08X\r\n", offset);
        ret = target_flash_erase_sector(&flash, offset);
        if(ret)
        {
            ERROR_TRACE("erase PFlash failed\r\n");
        }
        offset += Algorithm.sector_size;
        LED_GREEN_TOGGLE();
    }
    
    /* download program flash */
    len = f_pimage.fsize;
    offset = 0;
    
    while(offset < len)
    {
        printf("writting pflash addr:0x%08X\r\n", offset);
        
        error = f_read(&f_pimage, ImageBuf, flash.ram_to_flash_bytes_to_be_written, &br);
        if(error != FR_OK)
        {
            ERROR_TRACE("read file error");
        }
        
        ret = target_flash_program_page(&flash, offset, ImageBuf, br);
        offset += br;
        
        if(ret)
        {
            ERROR_TRACE("target_flash_program_page failed\r\n");
        }
        LED_GREEN_TOGGLE();
    }
    
    f_close(&f_pimage);

    /* process data flash */
    if(IsDFlashImageExist == true)
    {
        /* erase data flash sector */
        offset = 0;
        len = f_dimage.fsize;
        
        while(offset < (len + Algorithm.sector_size))
        {
            printf("erase dflash addr:0x%08X\r\n", offset);
            ret = target_flash_erase_sector(&flash, offset | 0x10000000 | (1<<23));
            if(ret)
            {
                ERROR_TRACE("erase DFlash failed\r\n");
            }
            offset += Algorithm.sector_size;
            LED_GREEN_TOGGLE();
        }
    
        /* download data flash */
        len = f_dimage.fsize;
        offset = 0;
    
        while(offset < len)
        {
            printf("writting dflash addr:0x%08X\r\n", offset);
            
            error = f_read(&f_dimage, ImageBuf, flash.ram_to_flash_bytes_to_be_written, &br);
            if(error != FR_OK)
            {
                ERROR_TRACE("read file error");
            }
            
            ret = target_flash_program_page(&flash, offset | 0x10000000 | (1<<23), ImageBuf, br);
            offset += br;
            
            if(ret)
            {
                ERROR_TRACE("target_flash_program_page failed\r\n");
            }
            LED_GREEN_TOGGLE();
        }
    
        f_close(&f_dimage);
    }
    
    /* release core and run target */
    SWJ_WriteAP(0x01000004, 0x00);
    val = 0;
    SWJ_WriteMem(DBG_EMCR, (uint8_t*)&val, sizeof(val));
    SWJ_SetTargetState(RESET_RUN);
    
    SWD_PinDeInit();
    printf("All operation complete\r\n");
    printf("Press reset pin for next download...\r\n");
    
    while(1)
    {
        LED_GREEN_TOGGLE();
        delay(500);
    }
}


void PIT0_IRQHandler(void)
{
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    GPIOC->PTOR |= (1<<10);
}
