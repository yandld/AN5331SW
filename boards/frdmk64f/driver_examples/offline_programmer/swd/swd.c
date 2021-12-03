#include "swd.h"

#ifndef SWD_DEBUG
#define SWD_DEBUG		1
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

uint8_t SWD_TransferOnce (uint32_t request, uint32_t *data)
{
    uint32_t ack;
    uint32_t bit;
    uint32_t val;
    uint32_t parity;
  
    uint32_t n;  
  
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
    DDIR(0);   
    for (n = 1; n; n--)
    {     
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
                    SW_WRITE_BIT(1);    
                }         
                DDIR(1);
            }
            else    /* write data */
            {
                /* Turnaround */       
                for (n = 1; n; n--) {    SW_WRITE_BIT(1); }
           
                DDIR(1);
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
            DOUT(1);        
            return (ack);   
        case DAP_TRANSFER_WAIT:
        case DAP_TRANSFER_FAULT:
            /* WAIT or FAULT response */          
            if (0 && ((request & DAP_TRANSFER_RnW) != 0))
            {   
                for (n = 32+1; n; n--)
                {            
                    SW_WRITE_BIT(1);  /* Dummy Read RDATA[0:31] + Parity */    
                }         
            }
            /* Turnaround */         
            for (n = 1; n; n--) { SW_WRITE_BIT(1);}      
            DDIR(1);  
            if (0 && ((request & DAP_TRANSFER_RnW) == 0))
            {   
                DOUT(0);      
                for (n = 32+1; n; n--) {    SW_WRITE_BIT(1);  /* Dummy Write WDATA[0:31] + Parity */   };           
            }           
            DOUT(1);        
            return (ack);  
        default:
            break;
    }
    
    /* Protocol error */       
    for (n = 1 + 32 + 1; n; n--)
    {         
        SW_WRITE_BIT(1);      /* Back off data phase */   
    }
    DOUT(1);          
    return (ack); 
}


#define MAX_SWD_RETRY   4
uint8_t SWD_Transfer (uint32_t request, uint32_t *data)
{
    uint8_t i, ack;
    for (i = 0; i < MAX_SWD_RETRY; i++) {
        ack = SWD_TransferOnce(request, data);
        // if ack != WAIT
        if (ack != 0x02) {
            return ack;
        }
    }
    return ack;
}


static void SWJ_SendClock(uint32_t count, uint8_t swdio_logic)
{
    while(count--)
    {
        SW_WRITE_BIT((swdio_logic)?(1):(0));
    }
}

static void SWJ_SendData(uint16_t data)
{
    uint8_t i;
    
    for(i = 0; i < 16; i++)
    {
        SW_WRITE_BIT(((data & 0x1) == 1)?(1):(0));
	    data>>=1;
    }
}

uint8_t SWJ_JTAG2SWD(void)
{
    SWJ_SendClock(51, 1);
    SWJ_SendData(0xE79E);
    SWJ_SendClock(51, 1);
    SWJ_SendClock(3, 0);
    return 0;
}

uint8_t SWJ_ReadDP(uint8_t adr, uint32_t *val)
{
    uint32_t tmp_in;
    uint8_t ack;
    uint8_t err;

    tmp_in = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(adr);
    ack = SWD_Transfer(tmp_in, val);

    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}

uint8_t SWJ_WriteDP(uint8_t adr, uint32_t val)
{
    uint32_t req;
    uint8_t ack;
    uint8_t err;
    
    req = SWD_REG_DP | SWD_REG_W | SWD_REG_ADR(adr);
    ack = SWD_Transfer(req, &val);

    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}

/* Read access port register. */
uint8_t SWJ_ReadAP(uint32_t adr, uint32_t *val)
{
    uint8_t tmp_in, ack, err;

    uint32_t apsel = adr & APSEL;
    uint32_t bank_sel = adr & APBANKSEL;

    if(SWJ_WriteDP(DP_SELECT, apsel | bank_sel))
    {
        return 1;
    }

    tmp_in = SWD_REG_AP | SWD_REG_R | SWD_REG_ADR(adr);

    /* first dummy read */
    ack = SWD_Transfer(tmp_in, val);
    ack = SWD_Transfer(tmp_in, val);

    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}


uint8_t SWJ_WriteAP(uint32_t adr, uint32_t val)
{
    uint8_t req, ack, err;
    
    uint32_t apsel = adr & APSEL;
    uint32_t bank_sel = adr & APBANKSEL;

    /* write DP select */
    if(SWJ_WriteDP(DP_SELECT, apsel | bank_sel))
    {
        
    }

    /* write AP data */
    req = SWD_REG_AP | SWD_REG_W | SWD_REG_ADR(adr);
    ack = SWD_Transfer(req, &val);

    /* read DP buff */
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = SWD_Transfer(req, NULL);
    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    
    return err;
}

/* Read 32-bit word from target memory. */
// AP CSW register, base value
#define CSW_VALUE (CSW_RESERVED | CSW_MSTRDBG | CSW_HPROT | CSW_DBGSTAT | CSW_SADDRINC)


// Write target memory.
static uint8_t SWJ_WriteData(uint32_t addr, uint32_t data)
{
    uint8_t req, ack, err;

    SWJ_WriteAP(AP_TAR, addr);

    /* write data */
    req = SWD_REG_AP | SWD_REG_W | AP_DRW;
    ack = SWD_Transfer(req, &data);
    
    /* read DP buff */
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = SWD_Transfer(req, NULL);

    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}

static uint8_t SWJ_WriteMem8(uint32_t addr, uint8_t val)
{
    uint32_t tmp;
    uint8_t err;
    
    SWJ_WriteAP(AP_CSW, CSW_VALUE | CSW_SIZE8);

    tmp = val << ((addr & 0x03) << 3);
    err = SWJ_WriteData(addr, tmp);
    return err;
}

uint8_t SWJ_WriteMem32(uint32_t addr, uint32_t val)
{
    uint8_t err;
    
    SWJ_WriteAP(AP_CSW, CSW_VALUE | CSW_SIZE32);
    err = SWJ_WriteData(addr, val);
    return err;
}

// Write 32-bit word aligned values to target memory using address auto-increment.
// size is in bytes.
static uint8_t SWJ_WriteBlock(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t err, req;
    uint32_t size_in_words;
    uint32_t i, ack;

    if (len == 0)  return 0;
    err = 0;
    size_in_words = len/4;

    
    err += SWJ_WriteAP(AP_CSW, CSW_VALUE | CSW_SIZE32);
    if(err)
    {
        //SWD_TRACE("AP_CSW, CSW_VALUE | CSW_SIZE32 failed\r\n");
    }
    err += SWJ_WriteAP(AP_TAR, addr);
    
    if(err) return err;

    /* DRW write */
    req = SWD_REG_AP | SWD_REG_W | (3 << 2);
    for (i = 0; i < size_in_words; i++)
    {
        if (SWD_Transfer(req, (uint32_t *)buf) != DAP_TRANSFER_OK)
        {
            return 1;
        }
        buf += 4;
    }

    /* read DP buff */
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = SWD_Transfer(req, NULL);
    
    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return 0;
}

// Read target memory.
static uint8_t SWJ_ReadData(uint32_t addr, uint32_t *val)
{
    uint8_t req, ack, err;

    SWJ_WriteAP(AP_TAR, addr);

    /* read data */
    req = SWD_REG_AP | SWD_REG_R | AP_DRW;
    ack = SWD_Transfer(req, val);

    /* dummy read */
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = SWD_Transfer(req, val);
    
    (ack == DAP_TRANSFER_OK)?(err = 0):(err = 1);
    return err;
}

static uint8_t SWJ_ReadMem32(uint32_t addr, uint32_t *val)
{
    uint8_t err;
    SWJ_WriteAP(AP_CSW, CSW_VALUE | CSW_SIZE32);
    err = SWJ_ReadData(addr, val);
    return err;
}

static uint8_t SWJ_ReadMem8(uint32_t addr, uint8_t *val)
{
    uint32_t tmp;
    uint8_t err;
    
    SWJ_WriteAP(AP_CSW, CSW_VALUE | CSW_SIZE8); 
    
    err = SWJ_ReadData(addr, &tmp);
    
    *val = (uint8_t)(tmp >> ((addr & 0x03) << 3));
    
    return err;
}

// Read 32-bit word aligned values from target memory using address auto-increment.
// size is in bytes.
static uint8_t SWJ_ReadBlock(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t err, req;
    uint32_t size_in_words;
    uint32_t i, ack;

    if (len == 0)  return 0;
    err = 0;
    size_in_words = len/4;

    
    err += SWJ_WriteAP(AP_CSW, CSW_VALUE | CSW_SIZE32);
    err += SWJ_WriteAP(AP_TAR, addr);
    
    if(err) return err;

    req = SWD_REG_AP | SWD_REG_R | (3 << 2);
    
    /* dummy read */
    if (SWD_Transfer(req, (uint32_t *)buf) != 0x01)
    {
        return 1;
    }
    
    for (i = 0; i < size_in_words; i++)
    {
        if (SWD_Transfer(req, (uint32_t *)buf) != DAP_TRANSFER_OK)
        {
            return 1;
        }
        buf += 4;
    }

    /* read DP buff */
    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = SWD_Transfer(req, NULL);
    
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
        SWJ_WriteMem8(address, *data);
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

        if (SWJ_WriteBlock(address, data, n))
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
        SWJ_WriteMem8(address, *data);

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
        SWJ_ReadMem8(address, data);
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

        if (SWJ_ReadBlock(address, data, n))
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
        SWJ_ReadMem8(address, data);
        address++;
        data++;
        size--;
    }

    return 0;
}

uint8_t SWJ_ReadMem(uint32_t addr, uint8_t *buf, uint32_t len)
{
    return swd_read_memory(addr, buf, len);
}

uint8_t SWJ_WriteMem(uint32_t addr, uint8_t *buf, uint32_t len)
{
    return swd_write_memory(addr, buf, len);
}


#define MAX_TIMEOUT 100
static uint8_t SWJ_WaitUntilHalted(void)
{
    // Wait for target to stop
    uint32_t val, i, timeout = MAX_TIMEOUT;
    for (i = 0; i < timeout; i++)
    {

        if (SWJ_ReadMem32(DBG_HCSR, &val))
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




/* Debug mailbox AP registers */
#define DEBUGMB_CSW  0x02000000
#define DEBUGMB_REQ  0x02000004
#define DEBUGMB_RET  0x02000008
#define DEBUGMB_ID   0x020000FC

#define DWT_COMP0               (0xE0001020)
#define DWT_FUNCTION0           (0xE0001028)
#define DWT_FUNCTION_MATCH      (0x4 << 0)
#define DWT_FUNCTION_ACTION     (0x1 << 4)
#define DWT_FUNCTION_DATAVSIZE  (0x2 << 10)

uint8_t SWJ_SetTargetState(TARGET_RESET_STATE state)
{
    uint32_t val, i, id, err;
    switch (state)
    {
        case RESET_HOLD:
            TRST(0);
            break;
        case RESET_RUN:
            TRST(0);
            DELAY_US(20*1000);
            TRST(1);
            DELAY_US(20*1000);
            break;
        case RESET_PROGRAM:
            TRST(0);
            DELAY_US(20);
            TRST(1);
            DELAY_US(200*100);

            SWJ_InitDebug();
        
        err = SWJ_WriteMem32(DBG_HCSR, DBGKEY | C_DEBUGEN);
        DELAY_US(200*100);

        err = SWJ_WriteAP(DEBUGMB_CSW, 0x21);
        DELAY_US(200*100);
        err = SWJ_ReadAP(DEBUGMB_CSW, &val);
        DELAY_US(200*100);
            
        err = SWJ_WriteAP(DEBUGMB_REQ, 0x07);
        err = SWJ_ReadAP(DEBUGMB_CSW, &val);
        DELAY_US(200*100);


        err = SWJ_ReadAP(DEBUGMB_ID, &id);
        printf("DEBUGMB_ID: %d  0x%X\r\n", err, id);
        
        SWJ_WriteMem32(0xE0002008, 0x215|1);            // Program FPB Comparator 0 with reset handler address
        SWJ_WriteMem32(0xE0002000, 0x00000003);         // Enable FPB


        // TRST(1);
        // Enable halt on reset
        SWJ_WriteMem32(DBG_EMCR, VC_CORERESET);
        DELAY_US(200*100);
            
        /* RESET using mailbox */ 
        err = SWJ_WriteAP(DEBUGMB_CSW, 0x20);
        DELAY_US(200*100);
          //SWJ_WriteMem32(NVIC_AIRCR, VECTKEY | SYSRESETREQ);

            break;
        case RESET_RUN_WITH_DEBUG:
            TRST(0);
            DELAY_US(20);
        
            SWJ_InitDebug();
        
            // Enable debug
            SWJ_WriteMem32(DBG_HCSR, DBGKEY | C_DEBUGEN);
        
            // Reset again
            TRST(0);
            DELAY_US(2*100);
            TRST(1);
        default:
            break;
    }
    return 0;
}

static uint8_t SWJ_WriteCoreReg(uint32_t n, uint32_t val)
{
    int i = 0, timeout = 100;
    
    SWJ_WriteMem32(DCRDR, val);

    SWJ_WriteMem32(DCRSR, n | REGWnR);

    // wait for S_REGRDY
    for (i = 0; i < timeout; i++)
    {
        SWJ_ReadMem32(DHCSR, &val);

        if (val & S_REGRDY)
        {
            return 0;
        }
    }

    return 1;
}

static uint8_t SWJ_ReadCoreReg(uint32_t n, uint32_t *val)
{
    int i = 0, timeout = 100, err;
    if (SWJ_WriteMem32(DCRSR, n))
        return 1;
    
    // wait for S_REGRDY
    for (i = 0; i < timeout; i++)
    {

        if (SWJ_ReadMem32(DHCSR, val))
            return 1;

        if (*val & S_REGRDY)
            break;
    }

    if (i == timeout)
        return 1;

    err = SWJ_ReadMem32(DCRDR, val);

    return err;
}

uint8_t swd_write_debug_state(DEBUG_STATE *state)
{
    uint32_t i, status;

    SWJ_WriteDP(DP_SELECT, 0);

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
    SWJ_WriteMem32(DBG_HCSR, DBGKEY | C_DEBUGEN);

    // check status
    SWJ_ReadDP(DP_CTRL_STAT, &status);
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
        
    if(SWJ_WaitUntilHalted())
    {
        SWD_TRACE("SWJ_WaitUntilHalted failed\r\n");
        return 1;
    }
        
    if(SWJ_ReadCoreReg(0, &state.r[0]))
    {
        SWD_TRACE("SWJ_ReadCoreReg failed\r\n");
        return 1;
    }
        

    // Flash functions return 0 if successful.
    if (state.r[0] != 0)
    {
        SWD_TRACE("resutlt failed:0x%X\r\n", state.r[0]);
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
    SWJ_WriteAP(MDM_CTRL, 0x00);
    
    /* clear VC_CORERESET */
    val = 0;
    ret = SWJ_WriteMem(DBG_EMCR, (uint8_t*)&val, sizeof(val));
    SWJ_SetTargetState(RESET_RUN);
    return ret;
}

uint8_t SWJ_InitDebug(void)
{
    uint32_t tmp = 0;
    uint32_t val;
    
    SWJ_JTAG2SWD();
    if(SWJ_ReadDP(DP_IDCODE, &val))
    {
        return 1;
    }
        
    
    SWJ_WriteDP(DP_ABORT, STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);

    /* Ensure CTRL/STAT register selected in DPBANKSEL */
    SWJ_WriteDP(DP_SELECT, 0);

    /* Power ups */
    SWJ_WriteDP(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ);

    do
    {
        if(!SWJ_ReadDP(DP_CTRL_STAT, &tmp))
        {
            return 0;
        }
    } while ((tmp & (CDBGPWRUPACK | CSYSPWRUPACK)) != (CDBGPWRUPACK | CSYSPWRUPACK));

    SWJ_WriteDP(DP_CTRL_STAT, CSYSPWRUPREQ | CDBGPWRUPREQ | TRNNORMAL | MASKLANE);

    return 0;
}

