

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
    /*                                   WDOG_BASE    WDOG_TYPE     SEC_SIZE  PGM CMD */
    {"xxDN256/512",                 0x40052000u,    WDOG_TYPE_16,    2048,   PGM4},
    {"K60DN512",                    0x40052000u,    WDOG_TYPE_16,    2048,   PGM4},
    {"KL28",                        0x40076000u,    WDOG_TYPE_32,    2048,   PGM4},
    {"MK20DX128",                   0x40052000u,    WDOG_TYPE_16,    1024,   PGM4},
    {"KE15Z256",                    0x40052000u,    WDOG_TYPE_32,    2048,   PGM8},
    {"KL0/1/26/27",                 0x40047000u,    WDOG_TYPE_COP,   1024,   PGM4},
    {"xxFN1M",                      0x40052000u,    WDOG_TYPE_16,    4096,   PGM8},
    {"xxFN256",                     0x40052000u,    WDOG_TYPE_16,    4096,   PGM4},
    {"KE02",                        0x40052000u,    WDOG_TYPE_8,     512,    PGM4},
    {"LPC802",                        0,    0,  1024,    PGM8},
};

/* buffer size (in byte) for read/write operations */
extern  target_flash_t flash_main;
extern  target_flash_t flash_eep;
#define TARGET_PFLASH_IAMGE_PATH        "/PIMAGE.BIN"
#define TARGET_TRIM_IAMGE_PATH          "/TRIM.BIN"
#define TARGET_EEP_IAMGE_PATH          "/KE02_EEP.BIN"
#define MAX_SECTOR_SIZE                 (1024)
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
static FIL f_trim_image;
static FIL f_ke_eep_image;
static FIL f_config;   /* config file */
static uint8_t ImageBuf[MAX_SECTOR_SIZE];   /* Read buffer, maxed sector size on Kinetis*/
static AlgorithmParams_t Algorithm;
static uint32_t FlashOption = 0;
static uint32_t KE_TRIM = 0;
static uint32_t KE_EEP = 0;
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
    PORTB->PCR[10] = PORT_PCR_MUX(1);
    GPIOB->PDDR |= (1<<10);
    
    /* SWCLK */
    PORTB->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOB->PDDR |= (1<<2);
    
    /* SWDIO */
    PORTB->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;
    GPIOB->PDDR |= (1<<3);
    
    SW_PinInit();
    
    TRST = 1;
    swd_init_debug();

    
    uint32_t id;
    swd_read_dp(DP_IDCODE, &id);
    printf("DP-IDR:0x%X\r\n", id);
    
    swd_read_ap(0x000000FC, &id);
    printf("AHB_AP_IDR:0x%X\r\n", id);

//    swd_read_ap(0x010000FC, &id);
//    printf("MDM-AP ID:0x%X\r\n", id);
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

static uint32_t program_image(FIL *f, target_flash_t *flash, uint32_t start_addr, uint32_t sec_size)
{
    int i;
    uint32_t len, ret, error;
    UINT br;
    uint32_t offset;
    uint32_t page_size = flash->ram_to_flash_bytes_to_be_written;
    
    /* inject flash algorithm */
    ret = target_flash_init(flash, 0, 0, 0, 0);
    if(ret)
    {
        ERROR_TRACE("dowloading flash algorithm failed");
    }
    
    delay(1);
    
    /* erase program flash sector */
    offset = 0;
    len = f->fsize;
    
    while(offset < len)
    {
        printf("erase addr:0x%08X\r\n", offset + start_addr);
        ret = target_flash_erase_sector(flash, offset + start_addr);
        if(ret)
        {
            ERROR_TRACE("erase flash failed\r\n");
        }
        offset += sec_size;
        LED_GREEN_TOGGLE();
    }
    
    delay(1);
    
    /* download program flash */
    len = f->fsize;
    offset = 0;
    
    uint8_t succ = 0;
    
    while(offset < len)
    {
        error = f_read(f, ImageBuf, flash->ram_to_flash_bytes_to_be_written, &br);
        if(error != FR_OK)
        {
            ERROR_TRACE("read file error");
        }
        
        printf("programing addr:0x%08X, size:%d\r\n", offset + start_addr, br);
        
        ret = target_flash_program_page(flash, offset + start_addr, ImageBuf, br);
        offset += br;
        
        
        if(ret)
        {
            ERROR_TRACE("target_flash_program_page failed\r\n");
        }
        LED_GREEN_TOGGLE();
    }
    
//    /* varify */
//    TRST = 0;
//    DELAY_US(20*100);
//    TRST = 1;
//    DELAY_US(20*100);
//    
//    set_swd_speed(20);
//    swd_set_target_state(RESET_PROGRAM);

//    /* inject flash algorithm */
//    ret = target_flash_init(flash, 0, 0, 0, 0);
//    if(ret)
//    {
//        ERROR_TRACE("dowloading flash algorithm failed");
//    }
    
//    f_lseek(f, 0);
//    offset = 0;
//    while(offset < len)
//    {
//        error = f_read(f, ImageBuf, flash->ram_to_flash_bytes_to_be_written, &br);
//        if(error != FR_OK)
//        {
//            ERROR_TRACE("read file error");
//        }
//        
//        static uint8_t buf[64];
//        memset(buf, 0xFF, 64);
//        swd_read_memory(offset + start_addr, buf, br);

//        for(i=0; i<br; i++)
//        {
//            if(buf[i] != ImageBuf[i])
//            {
//                ERROR_TRACE("varify error\r\n");
//            }
//        }

//        offset += br;
//        LED_GREEN_TOGGLE();
//    }
    
}







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
    
    PRINTF("CoreClock:%dHz.\r\n", CLOCK_GetCoreSysClkFreq());
    PRINTF("init SD card...\r\n");
    
    delay(1);

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
        
        p = strstr((const char*)ImageBuf, "KE_EEP=");
        if(p)
        {
            KE_EEP = strtoul(p+strlen("KE_EEP="), 0, 0);
        }
        printf("KE_EEP:%d\r\n", KE_EEP);
    }
    
    
    read_image(TARGET_PFLASH_IAMGE_PATH, &f_pimage);
    
    if(KE_TRIM == 1)
    {
        read_image(TARGET_TRIM_IAMGE_PATH, &f_trim_image);
    }
    
    if(KE_EEP == 1)
    {
        read_image(TARGET_EEP_IAMGE_PATH, &f_ke_eep_image);
    }
    
    delay(1);
    
//    /* unlock Kinetis */
//    printf("unlock...\r\n");
//    target_flash_unlock_sequence();
    
    /* prepare download */
    set_swd_speed(50);
    swd_set_target_state(RESET_PROGRAM);

////    /* program main image */
//    swd_set_target_state(RESET_PROGRAM);
    set_swd_speed(2);
    
    program_image(&f_pimage, &flash_main, 0x00000000, Algorithm.sector_size);
    f_close(&f_pimage);
    

    set_swd_speed(50);
    /* release core and run target */
    swd_write_ap(0x01000004, 0x00);
    val = 0;
    swd_write_memory(DBG_EMCR, (uint8_t*)&val, sizeof(val));
    swd_set_target_state(RESET_RUN);
    
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
