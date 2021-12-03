

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
    /*                                WDOG_BASE    WDOG_TYPE     SEC_SIZE  PGM CMD */
    {"xxDN256/512",                 0x40052000u,    WDOG_TYPE_16,    2048,   PGM4},
    {"K60DN512",                    0x40052000u,    WDOG_TYPE_16,    2048,   PGM4},
    {"KL28",                        0x40076000u,    WDOG_TYPE_32,    2048,   PGM4},
    {"MK20DX128",                   0x40052000u,    WDOG_TYPE_16,    1024,   PGM4},
    {"KE15Z256",                    0x40052000u,    WDOG_TYPE_32,    2048,   PGM8},
    {"KL0/1/26/27",                 0x40047000u,    WDOG_TYPE_COP,   1024,   PGM4},
    {"xxFN1M",                      0x40052000u,    WDOG_TYPE_16,    4096,   PGM8},
    {"xxFN256",                     0x40052000u,    WDOG_TYPE_16,    4096,   PGM4},
    {"KE06",                        0x40052000u,    WDOG_TYPE_8,     512,    PGM4},
    {"LPC55S69",                    0x40052000u,    WDOG_TYPE_8,     512,    PGM4},
    {"LPC55S69",                    0x40052000u,    WDOG_TYPE_8,     512,    PGM4},
};

/* buffer size (in byte) for read/write operations */
extern const target_flash_t flash;
#define TARGET_PFLASH_IAMGE_PATH        "/PIMAGE.BIN"
#define TARGET_TRIM_IAMGE_PATH          "/TRIM.BIN"
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

static FATFS g_fileSystem; /* File system object */
static FIL f_pimage;   /* pflash image file */
static FIL f_trim_image;   /* dflash image file */
static FIL f_config;   /* config file */
static uint8_t ImageBuf[MAX_SECTOR_SIZE];   /* Read buffer, maxed sector size on Kinetis*/
static AlgorithmParams_t Algorithm;
static uint32_t FlashOption = 0;
static uint32_t KE_TRIM = 0;
uint8_t trim_val = 0x4C;
    
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
            __NOP();
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
    PORTB->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOB->PDDR |= (1<<2);
    
    /* SWDIO */
    PORTB->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOB->PDDR |= (1<<3);
    
    SW_PinInit();
    
    TRST(1);
    SWJ_InitDebug();
    
    uint32_t id;
    
    id = 0;
    SWJ_ReadDP(DP_IDCODE, &id);
    printf("DP-IDR:0x%X\r\n", id);
    
    id = 0;
    SWJ_ReadAP(0x000000FC, &id);
    printf("AHB_AP_IDR:0x%X\r\n", id);

}

void SWD_PinDeInit(void)
{
    GPIOB->PDDR &= ~(1<<10);
    
    /* SWCLK */
    GPIOB->PDDR &= ~(1<<2);
    
    /* SWDIO */
    GPIOB->PDDR &= ~(1<<3);
}

static uint32_t read_image(const char *path, FIL *f)
{
    uint32_t error;
    
    error = f_open(f, path, FA_READ);
    if(error != FR_OK)
    {
        printf("cannot find %s!\r\n", path);
        LED_BLUE_ON();
        while(1);
    }
    else
    {
        PRINTF("image find %s, size:%d\r\n", path, f->fsize);
    }
}

#define RT_ALIGN_DOWN(size, align)      ((size) & ~((align) - 1))

static uint32_t program_image(FIL *f)
{
    uint32_t len, ret, error;
    UINT br;
    uint32_t offset;
    uint32_t start_addr = 0;
    uint32_t page_size = flash.ram_to_flash_bytes_to_be_written;
    
    /* inject flash algorithm */
    ret = target_flash_init(&flash,  0, 0, 0, 0);
    if(ret)
    {
        ERROR_TRACE("download flash algorithm failed");
    }
    
    
    /* erase program flash sector */
    offset = 0;
    len = f->fsize;
    
    while(offset <= RT_ALIGN_DOWN((len + Algorithm.sector_size), Algorithm.sector_size))
    {
        if(offset != 128*1024)
        {
            printf("erase addr:0x%08X\r\n", offset);
            ret = target_flash_erase_sector(&flash, offset);
            if(ret)
            {
                ERROR_TRACE("erase flash failed\r\n");
            }
            offset += Algorithm.sector_size;
            LED_GREEN_TOGGLE();
        }
        
        if(offset == 128*1024)
        {
            break;
        }
    }
    
    /* download program flash */
    len = f->fsize;
    offset = 0;
    
    while(offset < len)
    {
        //printf("writting pflash addr:0x%08X\r\n", offset);
        printf(">");
        
        error = f_read(f, ImageBuf, flash.ram_to_flash_bytes_to_be_written, &br);
        if(error != FR_OK)
        {
            ERROR_TRACE("read file error");
        }
        
        if(KE_TRIM == 1)
        {
            const uint32_t trim_addr = 0x3FF;

            if (((offset + start_addr) <= trim_addr) && ((offset + start_addr) + page_size) > trim_addr)
            {
                printf("file image addr:0x%X offset:0x%X, val:0x%X\r\n", trim_addr, offset, ImageBuf[trim_addr - offset]);
                ImageBuf[trim_addr - offset] = trim_val;
                ImageBuf[trim_addr - offset - 1] = 0x00;
                printf("new value:address:0x%X val:0x%X\r\n", trim_addr, ImageBuf[trim_addr - offset]);
            }
        }
        
        ret = target_flash_program_page(&flash, offset, ImageBuf, br);
        offset += br;
        
        if(ret)
        {
            ERROR_TRACE("target_flash_program_page failed\r\n");
        }
        LED_GREEN_TOGGLE();
    }
}



int main(void)
{
    FRESULT error;
    uint32_t val, i;
    const TCHAR driverNumberBuffer[3U] = {SDDISK + '0', ':', '/'};

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    MPU_Enable(MPU, false);
    LED_RED_INIT(1);
    LED_GREEN_INIT(1);
    LED_BLUE_INIT(1);
    
    PRINTF("CoreClock:%dHz.\r\n", CLOCK_GetCoreSysClkFreq());
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
        char *p = NULL;
        p = strstr((const char*)ImageBuf, "FLASH_OPTION=");
        if(p)
        {
            FlashOption = strtoul(p+strlen("FLASH_OPTION="), 0, 0);
        }

        printf("FlashOption:%d\r\n", FlashOption);
        Algorithm = AlgorithmTbl[FlashOption];
        printf("\r\n%s WdogType:%d WdogBaseAddr:0x%08X, SectorSize:%d  FlashCmd:%d\r\n", Algorithm.name, Algorithm.wdog_type, Algorithm.wodg_base, Algorithm.sector_size, Algorithm.pgm);
        
        p = strstr((const char*)ImageBuf, "KE_TRIM=");
        if(p)
        {
            KE_TRIM = strtoul(p+strlen("KE_TRIM="), 0, 0);
        }
        printf("KE_TRIM:%d\r\n", KE_TRIM);
    }
    
    
    read_image(TARGET_PFLASH_IAMGE_PATH, &f_pimage);
  //  read_image(TARGET_TRIM_IAMGE_PATH, &f_trim_image);
    

    set_swd_speed(2);
    /* prepare download */
    SWJ_SetTargetState(RESET_PROGRAM);
    

    

    
   
    program_image(&f_pimage);
    f_close(&f_pimage);
    
    
    set_swd_speed(50);
    /* release core and run target */
    SWJ_WriteAP(0x01000004, 0x00);
    val = 0;
    SWJ_WriteMem(DBG_EMCR, (uint8_t*)&val, sizeof(val));
    SWJ_SetTargetState(RESET_RUN);
    
    SWD_PinDeInit();
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
