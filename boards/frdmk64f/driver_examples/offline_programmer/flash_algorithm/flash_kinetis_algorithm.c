/* 
    Kinetis Flash algorithm
 */

#include "flash_kinetis.h"

/* 
    This algorithm is used for programming Kinetis, it's modified by CMSIS-DAP flash algorithm and add 4 parameters for deal with different flash type and wdog
    it will use target MCU's RAM @0x20000000 - 0x20000600, so this space must be a valid RAM area.
   
    alex.yang@nxp.com
*/

static const uint32_t KIENTIS_FLM[] = {
    0xE00ABE00, 0x062D780D, 0x24084068, 0xD3000040, 0x1E644058, 0x1C49D1FA, 0x2A001E52, 0x4770D1F2,
    0x49564857, 0x49578041, 0x21008041, 0x808143c9, 0x718121ff, 0x210171c1, 0x21207041, 0x49527001,
    0x70c82010, 0x4951206f, 0x60c80280, 0x47702000, 0x47702000, 0x484cb530, 0x71412130, 0x70422200,
    0x72412109, 0x23017202, 0x72427043, 0x24807202, 0x79417144, 0xd5fc0609, 0x06c97941, 0x2008d501,
    0x7941bd30, 0x42292533, 0x7042d119, 0x72412106, 0x70437202, 0x72412104, 0x7201210c, 0x70412102,
    0x724121ff, 0x22037201, 0x72417042, 0x720121fe, 0x79417144, 0xd5fc0609, 0x40287940, 0x2001d0e0,
    0x4931bd30, 0x714a2230, 0x704a2200, 0x724a220a, 0x0e520242, 0x2201720a, 0x0a02704a, 0x7208724a,
    0x71482080, 0x06007948, 0x7948d5fc, 0xd50106c0, 0x47702008, 0x21337948, 0xd0fa4008, 0x47702001,
    0x2400b570, 0x4b201dc9, 0xe03208cd, 0x71592130, 0x70592100, 0x72592106, 0x72190c01, 0x70592101,
    0x72590a01, 0x21027218, 0x78517059, 0x78117259, 0x21037219, 0x78d17059, 0x78917259, 0x21047219,
    0x79517059, 0x79117259, 0x21057219, 0x79d17059, 0x79917259, 0x21807219, 0x79597159, 0xd5fc0609,
    0x26337959, 0xd0014231, 0xbd702001, 0x30083208, 0x42a51c64, 0x2000d8ca, 0x0000bd70, 0x000020c5,
    0x40052000, 0x000028d9, 0x40020000, 0xf0003000, 0x00000000
};

 const target_flash_t flash = {
    0x20000021, // Init
    0x20000051, // UnInit
    0x20000055, // EraseChip
    0x200000c3, // EraseSector
    0x20000101, // ProgramPage


    // breakpoint = RAM start + 1
    // RSB : base address is address of Execution Region PrgData in map file
    //       to access global/static data
    // RSP : Initial stack pointer
    {
        0x20000001, // breakpoint instruction address
        0x20000000 + 0x20 + 0x600,  // static base register value (image start + header + static base offset(static varibles))
        0x20000900  // initial stack pointer
    },

    0x20000200, // program_buffer, any valid RAM location with +512 bytes of headroom
    0x20000000, // algo_start, start of RAM
    sizeof(KIENTIS_FLM), // algo_size, size of array above
    KIENTIS_FLM,  // image, flash algo instruction array
    512        // ram_to_flash_bytes_to_be_written
};


