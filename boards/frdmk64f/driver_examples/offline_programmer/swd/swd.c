#include "swd.h"

#ifndef SWD_DEBUG
#define SWD_DEBUG		0
#endif

#if (SWD_DEBUG == 1)
#include <stdio.h>
#define SWD_TRACE	printf
#else
#define SWD_TRACE(...)
#endif

#define DCRDR 0xE000EDF8
#define DCRSR 0xE000EDF4
#define DHCSR 0xE000EDF0
#define REGWnR (1 << 16)


typedef struct
{
    uint32_t r[16];
    uint32_t xpsr;
} DEBUG_STATE;

uint8_t swd_transfer (uint32_t request, uint32_t *data)
{
    uint32_t ack;
    uint32_t bit;
    uint32_t val;
    uint32_t parity;
  
    uint32_t n;  
  
    SW_DIR(1);
    /* Packet Request */      
    parity = 0;  
    SW_WRITE_BIT(1);         /* Start Bit */
    bit = request >> 0;        
    SW_WRITE_BIT(bit);       /* APnDP Bit */
    parity += bit;
    bit = request >> 1;        
    SW_WRITE_BIT(bit);       /* RnW Bit */  
    parity += bit;
    bit = request >> 2;        
    SW_WRITE_BIT(bit);       /* A2 Bit */   
    parity += bit;
    bit = request >> 3;        
    SW_WRITE_BIT(bit);       /* A3 Bit */   
    parity += bit;
    SW_WRITE_BIT(parity);    /* Parity Bit */            
    SW_WRITE_BIT(0);         /* Stop Bit */ 
    SW_WRITE_BIT(1);         /* Park Bit */ 
   
    /* Turnaround */           
    SW_DIR(0);   
    for (n = 1; n; n--)
    {     
       // SW_CLOCK_CYCLE(); 
        SW_WRITE_BIT(1);        
    }
   
    /* Acknowledge response */ 
    bit = SW_READ_BIT();          
    ack  = bit << 0;           
    bit = SW_READ_BIT();          
    ack |= bit << 1;           
    bit = SW_READ_BIT();          
    ack |= bit << 2;           
   
    switch(ack)
    {
        case DAP_TRANSFER_OK:
            if (request & DAP_TRANSFER_RnW) /* read data */
            {
                val = 0;  
                parity = 0;            
                for (n = 32; n; n--)
                { 
                    bit = SW_READ_BIT();  /* Read RDATA[0:31] */      
                    parity += bit;       
                    val >>= 1;           
                    val  |= bit << 31;   
                }         
                bit = SW_READ_BIT();    /* Read Parity */           
                if ((parity ^ bit) & 1)
                {           
                    ack = DAP_TRANSFER_ERROR;         
                }               
                if (data) *data = val;
            
                /* Turnaround */       
                for (n = 1; n; n--)
                { 
                    //SW_CLOCK_CYCLE();   
                    SW_WRITE_BIT(1);
                }         
                SW_DIR(1);
            }
            else    /* write data */
            {
                /* Turnaround */       
                for (n = 1; n; n--) 
                {    
                    SW_WRITE_BIT(1);
                    //SW_CLOCK_CYCLE(); 
                }
           
                SW_DIR(1);
                /* Write data */       
                val = *data;           
                parity = 0;            
                for (n = 32; n; n--)
                { 
                    SW_WRITE_BIT(val); /* Write WDATA[0:31] */     
                    parity += val;       
                    val >>= 1;           
                }         
                SW_WRITE_BIT(parity);/* Write Parity Bit */      
            }
            
            /* Idle cycles */               
            TDOUT = 1;      
            return (ack);   
        case DAP_TRANSFER_WAIT:
        case DAP_TRANSFER_FAULT:
        case 0x07:
            /* WAIT or FAULT response */          
            if (0 && ((request & DAP_TRANSFER_RnW) != 0))
            {   
                for (n = 32+1; n; n--)
                {        
                    SW_WRITE_BIT(1);                    
                    //SW_CLOCK_CYCLE();  /* Dummy Read RDATA[0:31] + Parity */    
                }         
            }
            /* Turnaround */         
            for (n = 1; n; n--)
            { 
                //SW_CLOCK_CYCLE();
                SW_WRITE_BIT(1);
            }
            SW_DIR(1);  
            if (0 && ((request & DAP_TRANSFER_RnW) == 0))
            {   
                TDOUT = 0;      
                for (n = 32+1; n; n--) 
                {  
                   // SW_CLOCK_CYCLE();  /* Dummy Write WDATA[0:31] + Parity */
                    SW_WRITE_BIT(1);
                };           
            }           
            TDOUT = 1;        
            return (ack);  
        default:
            break;
    }
    
    /* Protocol error */       
    for (n = 1 + 32 + 1; n; n--)
    {
        SW_WRITE_BIT(1);
        //SW_CLOCK_CYCLE();      /* Back off data phase */   
    }
    TDOUT = 1;          
    return (ack); 
}


#define MAX_SWD_RETRY   10
uint8_t swd_transfer_retry (uint32_t request, uint32_t *data)
{
    uint8_t ack;
    uint32_t i;
    for (i = 0; i < MAX_SWD_RETRY; i++)
    {
        __disable_irq();
        ack = swd_transfer(request, data);
        __enable_irq();
        // if ack != WAIT
        if (ack != 0x02)
        {
            return ack;
        }
    }
    return ack;
}


void swd_sequence(uint32_t count, uint8_t *data)
{
    volatile int i,j;
//    if((count % 8) != 0)
//    {
//        count = count + 8;
//    }
    
    for(i=0; i<count/8; i++)
    {
        for(j=0; j<8; j++)
        {
            SW_WRITE_BIT(((data[i] & 0x01) == 1)?(1):(0));
            data[i] >>= 1;
        }
    }
}



uint8_t JTAG2SWD(void)
{
    int i;
    uint8_t buf[8];
    
    /* 64 clock 1 */
    for(i=0; i<sizeof(buf); i++)
    {
        buf[i] = 0xFF;
    }
    swd_sequence(64, buf);
    
    /* 9E E9 */
    buf[0] = 0x9E;
    buf[1] = 0xE7;
    swd_sequence(16, buf);
    
    /* 64 clock 1 and 8 clock 0 */
    for(i=0; i<sizeof(buf); i++)
    {
        buf[i] = 0xFF;
    }
    swd_sequence(64, buf);
    
    buf[0] = 0x00;
    swd_sequence(8, buf);
    return 0;
}

uint8_t swd_read_dp(uint8_t adr, uint32_t *val)
{
    uint32_t tmp_in;
    uint8_t ack;
    uint8_t err;

    tmp_in = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(adr);
    ack = swd_transfer_retry(tmp_in, val);

    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}

uint8_t swd_write_dp(uint8_t adr, uint32_t val)
{
    uint32_t req;
    uint8_t ack;
    uint8_t err;
    
    req = SWD_REG_DP | SWD_REG_W | SWD_REG_ADR(adr);
    ack = swd_transfer_retry(req, &val);

    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}

/* Read access port register. */
uint8_t swd_read_ap(uint32_t adr, uint32_t *val)
{
    uint8_t tmp_in, ack, err;

    uint32_t apsel = adr & APSEL;
    uint32_t bank_sel = adr & APBANKSEL;

    if(swd_write_dp(DP_SELECT, apsel | bank_sel))
    {
        return 1;
    }

    tmp_in = SWD_REG_AP | SWD_REG_R | SWD_REG_ADR(adr);

    /* first dummy read */
    ack = swd_transfer_retry(tmp_in, val);
    ack = swd_transfer_retry(tmp_in, val);
    
    /* can also read RDBUFF as the second read, see V5.2: Read Buffer implementation and use on a SW-DP */
//    uint8_t req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
//    ack = swd_transfer_retry(req, val);
//    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);

    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}


uint8_t swd_write_ap(uint32_t adr, uint32_t val)
{
    uint8_t req, ack, err;
    
    uint32_t apsel = adr & APSEL;
    uint32_t bank_sel = adr & APBANKSEL;

    /* write DP select */
    if(swd_write_dp(DP_SELECT, apsel | bank_sel))
    {
        return 1;
    }

    /* write AP data */
    req = SWD_REG_AP | SWD_REG_W | SWD_REG_ADR(adr);
    ack = swd_transfer_retry(req, &val);

    /* read DP buff */
    /* If a DP read of the IDCODE or CTRL/STAT register, or a DP write to the ABORT register, is required immediately
after a sequence of AP writes, the software must first perform an access that the DP is able to stall. This ensures that
the write buffer is emptied before the DP register access is performed. If this is not done, WDATAERR might be set
to 1, and the buffered writes lost. */
    
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = swd_transfer_retry(req, NULL);
    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    
    return err;
}

/* Read 32-bit word from target memory. */
// AP CSW register, base value
#define CSW_VALUE (CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC)


// Write target memory.
static uint8_t swd_write_data(uint32_t addr, uint32_t data)
{
    uint8_t req, ack, err;

    swd_write_ap(AP_TAR, addr);

    /* write data */
    req = SWD_REG_AP | SWD_REG_W | AP_DRW;
    ack = swd_transfer_retry(req, &data);
    
    /* read DP buff */
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = swd_transfer_retry(req, NULL);

    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}

static uint8_t swd_write_byte(uint32_t addr, uint8_t val)
{
    uint32_t tmp;
    uint8_t err;
    
    swd_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE8);

    tmp = val << ((addr & 0x03) << 3);
    err = swd_write_data(addr, tmp);
    return err;
}

uint8_t swd_write_word(uint32_t addr, uint32_t val)
{
    uint8_t err;
    
    swd_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32);
    err = swd_write_data(addr, val);
    return err;
}

// Write 32-bit word aligned values to target memory using address auto-increment.
// size is in bytes.
static uint8_t swd_write_block(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t err, req;
    uint32_t size_in_words;
    uint32_t i, ack;

    if (len == 0)  return 0;
    err = 0;
    size_in_words = len/4;

    
    err += swd_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32);
    if(err)
    {
        //SWD_TRACE("AP_CSW, CSW_VALUE | CSW_SIZE32 failed\r\n");
    }
    err += swd_write_ap(AP_TAR, addr);
    
    if(err) return err;

    /* DRW write */
    req = SWD_REG_AP | SWD_REG_W | (3 << 2);
    for (i = 0; i < size_in_words; i++)
    {
        if (swd_transfer_retry(req, (uint32_t *)buf) != DAP_TRANSFER_OK)
        {
            return 1;
        }
        buf += 4;
    }

    /* read DP buff */
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = swd_transfer_retry(req, NULL);
    
    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return 0;
}

// Read target memory.
static uint8_t swd_read_data(uint32_t addr, uint32_t *val)
{
    uint8_t req, ack, err;

    swd_write_ap(AP_TAR, addr);

    /* read data */
    req = SWD_REG_AP | SWD_REG_R | AP_DRW;
    ack = swd_transfer_retry(req, val);

    /* dummy read */
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = swd_transfer_retry(req, val);
    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}

static uint8_t swd_read_word(uint32_t addr, uint32_t *val)
{
    uint8_t err;
    swd_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32);
    err = swd_read_data(addr, val);
    return err;
}

static uint8_t swd_read_byte(uint32_t addr, uint8_t *val)
{
    uint32_t tmp;
    uint8_t err;
    
    swd_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE8); 
    
    err = swd_read_data(addr, &tmp);
    
    *val = (uint8_t)(tmp >> ((addr & 0x03) << 3));
    
    return err;
}

// Read 32-bit word aligned values from target memory using address auto-increment.
// size is in bytes.
static uint8_t swd_read_block(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t err, req;
    uint32_t size_in_words;
    uint32_t i, ack;

    if (len == 0)  return 0;
    err = 0;
    size_in_words = len/4;

    
    err += swd_write_ap(AP_CSW, CSW_VALUE | CSW_SIZE32);
    err += swd_write_ap(AP_TAR, addr);
    
    if(err) return err;

    req = SWD_REG_AP | SWD_REG_R | (3 << 2);
    
    /* dummy read */
    if (swd_transfer_retry(req, (uint32_t *)buf) != 0x01)
    {
        return 1;
    }
    
    for (i = 0; i < size_in_words; i++)
    {
        if (swd_transfer_retry(req, (uint32_t *)buf) != DAP_TRANSFER_OK)
        {
            return 1;
        }
        buf += 4;
    }

    /* read DP buff */
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = swd_transfer_retry(req, NULL);
    
    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;   
}

#define TARGET_AUTO_INCREMENT_PAGE_SIZE    (0x400)

// Write unaligned data to target memory.
// size is in bytes.
uint8_t swd_write_memory(uint32_t address, uint8_t *data, uint32_t size)
{
    uint32_t n;
    // Write bytes until word aligned
    while ((size > 0) && (address & 0x3))
    {
        swd_write_byte(address, *data);
        address++;
        data++;
        size--;
    }

    // Write word aligned blocks
    while (size > 3) {
        // Limit to auto increment page size
        n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (address & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));
        if (size < n) {
            n = size & 0xFFFFFFFC; // Only count complete words remaining
        }

        if (swd_write_block(address, data, n))
        {
            SWD_TRACE("SWJ write block failed\r\n");
            return 1;
        }

        address += n;
        data += n;
        size -= n;
    }

    // Write remaining bytes
    while (size > 0) {
        swd_write_byte(address, *data);

        address++;
        data++;
        size--;
    }

    return 0;
}

// Read unaligned data from target memory.
// size is in bytes.
uint8_t swd_read_memory(uint32_t address, uint8_t *data, uint32_t size)
{
    uint32_t n;

    // Read bytes until word aligned
    while ((size > 0) && (address & 0x3))
    {
        swd_read_byte(address, data);
        address++;
        data++;
        size--;
    }

    // Read word aligned blocks
    while (size > 3)
    {
        // Limit to auto increment page size
        n = TARGET_AUTO_INCREMENT_PAGE_SIZE - (address & (TARGET_AUTO_INCREMENT_PAGE_SIZE - 1));
        if (size < n)
        {
            n = size & 0xFFFFFFFC; // Only count complete words remaining
        }

        if (swd_read_block(address, data, n))
        {
            return 1;
        }

        address += n;
        data += n;
        size -= n;
    }

    // Read remaining bytes
    while (size > 0)
    {
        swd_read_byte(address, data);
        address++;
        data++;
        size--;
    }

    return 0;
}




#define MAX_TIMEOUT 10000
static uint8_t SWJ_WaitUntilHalted(void)
{
    // Wait for target to stop
    uint32_t val, i, timeout = MAX_TIMEOUT;
    for (i = 0; i < timeout; i++)
    {

        if (swd_read_word(DBG_HCSR, &val))
        {
            return 1;
        }

        if (val & S_HALT)
        {
            return 0;
        }
    }
    return DAP_TRANSFER_ERROR;
}


#define MAX_TIMEOUT1 (20000)
static uint8_t swd_wait_until_halted(void)
{
    // Wait for target to stop
    uint32_t val, i, timeout = MAX_TIMEOUT1;
    for (i = 0; i < timeout; i++)
    {
        if (swd_read_word(DBG_HCSR, &val))
        {
            return 1;
        }

        if (val & S_RESET_ST)
        {
            return 1;
        }
        
        if (val & S_HALT)
        {
            return 0;
        }
    }
    return 1;
}

#define MAX_TIMEOUT2 (500)
static uint8_t swd_wait_until_halted2(void)
{
    // Wait for target to stop
    uint32_t val, i, timeout = MAX_TIMEOUT2;
    for (i = 0; i < timeout; i++)
    {
        if (swd_read_word(DBG_HCSR, &val))
        {
            return 1;
        }

        if (val & S_HALT)
        {
            return 0;
        }
    }
    return 1;
}

uint8_t swd_set_target_state(TARGET_RESET_STATE state)
{
    int ret;
    
    switch (state)
    {
        case RESET_HOLD:
            swd_hw_write_trst(0);
            break;
        case RESET_RUN:
            swd_hw_write_trst(0);
            swd_delay_us(20*1000);
            swd_hw_write_trst(1);
            swd_delay_us(20*1000);
            break;
        case RESET_PROGRAM:
            swd_hw_write_trst(0);
            swd_delay_us(20*1000);
            swd_hw_write_trst(1);
            swd_delay_us(20*1000);
        
            ret = swd_init_debug();
            if(ret)
            {
                swd_hw_write_trst(0);
                swd_delay_us(2*1000);
                ret = swd_init_debug();
                if(ret)
                {
                    SWD_TRACE("swd_init_debug error\r\n");
                    return ret;
                }
            }
            
            swd_hw_write_trst(1);
        
            // Enable debug
            ret = swd_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN | C_HALT);
            if(ret)
            {
                SWD_TRACE("C_DEBUGEN error\r\n");
                return ret;
            }
            
            // Enable halt on reset
            ret = swd_write_word(DBG_EMCR, VC_CORERESET);
            if(ret)
            {
                SWD_TRACE("VC_CORERESET error\r\n");
                return ret;
            }

            // Perform a soft reset, to due with hardware without a reset pin
            ret = swd_write_word(NVIC_AIRCR, VECTKEY | VECTRESET );
            if(ret)
            {
                SWD_TRACE("SYSRESETREQ error, continue...\r\n");
                //return ret;
            }
            
            ret = swd_wait_until_halted2();
            if(ret)
            {
                SWD_TRACE("soft reset failed, try hw reset\r\n");
                swd_hw_write_trst(0);
                swd_delay_us(20*1000);
                swd_hw_write_trst(1);
                swd_delay_us(20*1000);
                ret = swd_wait_until_halted2();
                if(ret)
                {
                    SWD_TRACE("hw reset also failed!\r\n");
                    return ret;
                }
                else
                {
                    SWD_TRACE("hw reset successully halted the core!\r\n");
                }
            }

            // Disable halt on reset
            ret = swd_write_word(DBG_EMCR, 0);
            if(ret)
            {
                SWD_TRACE("VC_CORERESET disable error\r\n");
                return ret;
            }
                
            break;
        case RESET_RUN_WITH_DEBUG:
            swd_hw_write_trst(0);
            swd_delay_us(20*1000);

            ret = swd_init_debug();
        
            // Enable debug
            ret = swd_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN);
            if(ret)
            {
                SWD_TRACE("C_DEBUGEN error\r\n");
                return ret;
            }
            
            // Reset again
            swd_hw_write_trst(0);
            swd_delay_us(20*1000);
            swd_hw_write_trst(1);
            swd_delay_us(20*1000);
            break;
        default:
            break;
    }
    return ret;
}

static uint8_t SWJ_WriteCoreReg(uint32_t n, uint32_t val)
{
    int i = 0, timeout = 10000;
    
    swd_write_word(DCRDR, val);

    swd_write_word(DCRSR, n | REGWnR);

    // wait for S_REGRDY
    for (i = 0; i < timeout; i++)
    {
        swd_read_word(DHCSR, &val);

        if (val & S_REGRDY)
        {
            return 0;
        }
    }

    return 1;
}

static uint8_t swd_read_core_register(uint32_t n, uint32_t *val)
{
    int i = 0, timeout = 100, err;
    if (swd_write_word(DCRSR, n))
        return 1;
    
    // wait for S_REGRDY
    for (i = 0; i < timeout; i++)
    {

        if (swd_read_word(DHCSR, val))
            return 1;

        if (*val & S_REGRDY)
            break;
    }

    if (i == timeout)
        return 1;

    err = swd_read_word(DCRDR, val);

    return err;
}

uint8_t swd_write_debug_state(DEBUG_STATE *state)
{
    uint32_t i, status;

    swd_write_dp(DP_SELECT, 0);

    // R0, R1, R2, R3
    for (i = 0; i < 4; i++)
    {
        SWJ_WriteCoreReg(i, state->r[i]);
    }
    // R9
    SWJ_WriteCoreReg(9, state->r[9]);
    // R13, R14, R15
    for (i=13; i<16; i++)
    {
        SWJ_WriteCoreReg(i, state->r[i]);
    }

    // xPSR
    SWJ_WriteCoreReg(16, state->xpsr);
    swd_write_word(DBG_HCSR, DBGKEY | C_DEBUGEN);

    // check status
    swd_read_dp(DP_CTRL_STAT, &status);
    if (status & (STICKYERR | WDATAERR))
    {
        SWD_TRACE("write debug states failed\r\n");
        return 1;
    }

    return 0;
}





uint8_t swd_flash_syscall_exec(const FLASH_SYSCALL *sysCallParam, uint32_t entry, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t arg4)
{
    DEBUG_STATE state;

    // Call flash algorithm function on target and wait for result.
    state.xpsr     = 0x01000000;          // xPSR: T = 1, ISR = 0
    state.r[0]     = arg1;                   // R0: Argument 1
    state.r[1]     = arg2;                   // R1: Argument 2
    state.r[2]     = arg3;                   // R2: Argument 3
    state.r[3]     = arg4;                   // R3: Argument 4

    state.r[9]     = sysCallParam->static_base;    // SB: Static Base

    state.r[13]    = sysCallParam->stack_pointer;  // SP: Stack Pointer
    state.r[14]    = sysCallParam->breakpoint;       // LR: Exit Point
    state.r[15]    = entry;                           // PC: Entry Point

    if (swd_write_debug_state(&state))
    {
        SWD_TRACE("swd_write_debug_status failed\r\n");
        return 1;
    }
        
    if(swd_wait_until_halted())
    {
        SWD_TRACE("swd_wait_until_halted timeout\r\n");
        return 1;
    }
        
    if(swd_read_core_register(0, &state.r[0]))
    {
        SWD_TRACE("swd_read_core_register failed\r\n");
        return 1;
    }
        

    // Flash functions return 0 if successful.
    if (state.r[0] != 0)
    {
        SWD_TRACE("fun result failed:0x%X\r\n", state.r[0]);
        return 1;
    }
    
    return 0;
}


#define MDM_CTRL                            (0x01000004) 

int SWD_Disconnect(void)
{
    int ret;
    uint32_t val;
    
    /* clear debug reqeust and system reset */
    swd_write_ap(MDM_CTRL, 0x00);
    
    /* clear VC_CORERESET */
    val = 0;
    ret = swd_write_memory(DBG_EMCR, (uint8_t*)&val, sizeof(val));
    swd_set_target_state(RESET_RUN);
    return ret;
}

uint8_t swd_init_debug(void)
{
    uint32_t tmp = 0;
    uint32_t val;
    
    JTAG2SWD();
    
    if(swd_read_dp(DP_IDCODE, &val))
    {
        SWD_TRACE("Read DP_IDCODE error\r\n"); 
        return 1;
    }
    else
    {
        SWD_TRACE("DPID:0x%08X\r\n", val);
    }
    
    if(swd_write_dp(DP_ABORT, STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR) != 0)
    {
        SWD_TRACE("DP DP_ABORT| STKCMPCLR error\r\n"); 
        return 1;
    }

    /* Ensure CTRL/STAT register selected in DPBANKSEL */
    swd_write_dp(DP_SELECT, 0);

    /* Power ups */
    if(swd_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ) != 0)
    {
        SWD_TRACE("DP CSYSPWRUPREQ error\r\n"); 
        return 1;
    }

    do
    {
        if(swd_read_dp(DP_CTRL_STAT, &tmp) != 0)
        {
            SWD_TRACE("DP DP_CTRL_STAT error\r\n"); 
            return 1;
        }
    } while ((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) != (CDBGPWRUPACK | CSYSPWRUPACK));

    swd_write_dp(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | TRNNORMAL | MASKLANE);
    
    swd_write_dp(DP_SELECT, 0);

    return 0;
}

