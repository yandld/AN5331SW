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
    0x70082010, 0x4951206f, 0x60c80280, 0x47702000, 0x47702000, 0x484cb530, 0x71812130, 0x70822200,
    0x72812109, 0x230172c2, 0x72827083, 0x248072c2, 0x79817184, 0xd5fc0609, 0x06c97981, 0x2008d501,
    0x7981bd30, 0x42292533, 0x7082d119, 0x72812106, 0x708372c2, 0x72812104, 0x72c1210c, 0x70812102,
    0x728121ff, 0x220372c1, 0x72817082, 0x72c121fe, 0x79817184, 0xd5fc0609, 0x40287980, 0x2001d0e0,
    0x4931bd30, 0x718a2230, 0x708a2200, 0x728a220a, 0x0e520242, 0x220172ca, 0x0a02708a, 0x72c8728a,
    0x71882080, 0x06007988, 0x7988d5fc, 0xd50106c0, 0x47702008, 0x21337988, 0xd0fa4008, 0x47702001,
    0x2400b570, 0x4b201dc9, 0xe03208cd, 0x71992130, 0x70992100, 0x72992106, 0x72d90c01, 0x70992101,
    0x72990a01, 0x210272d8, 0x78517099, 0x78117299, 0x210372d9, 0x78d17099, 0x78917299, 0x210472d9,
    0x79517099, 0x79117299, 0x210572d9, 0x79d17099, 0x79917299, 0x218072d9, 0x79997199, 0xd5fc0609,
    0x26337999, 0xd0014231, 0xbd702001, 0x30083208, 0x42a51c64, 0x2000d8ca, 0x0000bd70, 0x000020c5,
    0x40052000, 0x000028d9, 0x40020000, 0xf0003000, 0x00000000
};

  target_flash_t flash_main = {
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
        0x20000000 + 0x20 + 0x400,  // static base register value (image start + header + static base offset(static varibles))
        0x20000600  // initial stack pointer
    },

    0x20000200, // program_buffer, any valid RAM location with +512 bytes of headroom
    0x20000000, // algo_start, start of RAM
    sizeof(KIENTIS_FLM), // algo_size, size of array above
    KIENTIS_FLM,  // image, flash algo instruction array
    512        // ram_to_flash_bytes_to_be_written
};


