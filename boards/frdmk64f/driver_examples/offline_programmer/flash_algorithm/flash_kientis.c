#include <stdio.h>
#include "swd.h"
#include "flash_kinetis.h"

/* MDM-AP register address */
#define MDM_STATUS                          (0x01000000)
#define MDM_CTRL                            (0x01000004) 
#define MDM_IDR                             (0x010000FC)
#define KINETIS_KK_MCU_ID                   (0x001C0000)
#define KINETIS_KL_MCU_ID                   (0x001C0020)

/* return */
#define CH_OK                       (0)
#define CH_ERR                      (1)
#define CH_MESS_ERASE_DISABLED      (2)

/* better keep RST low before enter target_flash_unlock_sequence */
uint8_t target_flash_unlock_sequence(void)
{
    uint32_t val;
    
    /* verify the result */
    SWJ_ReadAP(MDM_IDR, &val);
    if ((val != KINETIS_KK_MCU_ID) && (val != KINETIS_KL_MCU_ID))
    {
        printf("not Kinetis!\r\n");
        return CH_ERR;
    }
   
    SWJ_ReadAP(MDM_STATUS, &val);
    
    /* flash in secured mode */
 //   if (val & (1 << 2))
    {
        printf("chip is secrued!\r\n");
        /* hold the device in reset */
        TRST(0);
        if(SWJ_WriteAP(MDM_CTRL, (1<<0)))
        {
            printf("launch mess erase failed\r\n");
            /* launch mess erase failed */
            return CH_ERR;
        }
        
        TRST(1);
        while (1)
        {
            /* wait until mass erase is started */
            if(SWJ_ReadAP(MDM_STATUS, &val))
            {
                /* read MDM_STATUS failed */
                return CH_ERR;
            }
            if (val & 1) break;
        }
        /* mass erase in progress */
        while (1)
        {
            /* keep reading until procedure is complete */
            if(SWJ_ReadAP(MDM_CTRL, &val))
            {
                printf("failed\r\n");
                 return CH_ERR;
            }
            if ((val & 0x01) == 0x00)
                break;
        }
        
        /* reset MDM-CTRL */
        SWJ_WriteAP(MDM_CTRL, 0);
    }
//    else
//    {
//        printf("device is not in secure state\r\n");
//    }
    
    /* halt MCU otherwise it loops in hard fault - WDOG reset cycle */
    TRST(0);
    return CH_OK;
}


uint8_t target_flash_init(const target_flash_t* flash, uint32_t wdog_base_addr, uint32_t wdog_type, uint32_t sector_size, uint32_t pgm_cmd)
{
    uint8_t ret;
    
    /* Download flash programming algorithm to target and initialise. */
    ret = SWJ_WriteMem(flash->algo_start, (uint8_t *)flash->image, flash->algo_size);
    if(ret)
    {
        /* inject flash algorithm failed */
        return CH_ERR;
    }
    
    ret = swd_flash_syscall_exec(&flash->sys_call_param, flash->init, 
                                wdog_base_addr,
                                wdog_type,
                                sector_size,
                                pgm_cmd );
    return ret;
}

uint8_t target_flash_erase_sector(const target_flash_t* flash, unsigned int addr)
{
    uint8_t ret;
    ret = swd_flash_syscall_exec(&flash->sys_call_param, flash->erase_sector, addr, 0, 0, 0);
    return ret;
}

uint8_t target_flash_erase_chip(const target_flash_t* flash)
{
    uint8_t ret;
    ret = swd_flash_syscall_exec(&flash->sys_call_param, flash->erase_chip, 0, 0, 0, 0);
    return ret;
}


// Check Flash Configuration Field bytes at address 0x400-0x40f to ensure that flash security
// won't be enabled.
//
// FCF bytes:
// [0x0-0x7]=backdoor key
// [0x8-0xb]=flash protection bytes
// [0xc]=FSEC:
//      [7:6]=KEYEN (2'b10 is backdoor key enabled, all others backdoor key disabled)
//      [5:4]=MEEN (2'b10 mass erase disabled, all other mass erase enabled)
//      [3:2]=FSLACC (2'b00 and 2'b11 factory access enabled, 2'b01 and 2'b10 factory access disabled)
//      [1:0]=SEC (2'b10 flash security disabled, all other flash security enabled)
// [0xd]=FOPT
// [0xe]=EEPROM protection bytes (FlexNVM devices only)
// [0xf]=data flash protection bytes (FlexNVM devices only)
//
// This function checks that:
// - FSEC does not disable mass erase or secure the device.
//
uint8_t security_bits_set(uint32_t addr, uint8_t *data, uint32_t size)
{
    const uint32_t fsec_addr = 0x40C;

    if ((addr <= fsec_addr) && (addr + size) > fsec_addr) {
        uint8_t fsec = data[fsec_addr - addr];

        // make sure we can unsecure the device or dont program at all
        if ((fsec & 0x30) == 0x20) {
            // Dont allow programming mass-erase disabled state
            return 1;
        }

        // Security is OK long as we can mass-erase (comment the following out to enable target security)
//        if ((fsec & 0x03) != 0x02) {
//            return 1;
//        }
    }

    return 0;
}


uint8_t target_flash_program_page(const target_flash_t* flash, uint32_t addr, uint8_t * buf, uint32_t size)
{
    uint32_t bytes_written = 0;

    // call a target dependent function to check if
    // we don't want to write specific bits (flash security bits, ...)
    if (security_bits_set(addr, buf, size) == 1)
    {
        return CH_MESS_ERASE_DISABLED;
    }

    // Program a page in target flash.
    if(SWJ_WriteMem(flash->program_buffer, buf, size))
    {
        /* write memory failed */
        return CH_ERR;
    }

    while(bytes_written < size)
    {
        if (swd_flash_syscall_exec(&flash->sys_call_param, flash->program_page, addr, flash->ram_to_flash_bytes_to_be_written, flash->program_buffer + bytes_written, 0))                      
        {
            return CH_ERR;
        }

        bytes_written += flash->ram_to_flash_bytes_to_be_written;
        addr += flash->ram_to_flash_bytes_to_be_written;
    }

    return CH_OK;
}


