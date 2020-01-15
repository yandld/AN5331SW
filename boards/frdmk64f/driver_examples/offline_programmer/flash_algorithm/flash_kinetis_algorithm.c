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
    0x4605b5f8, 0x4617460e, 0x4843461c, 0x60044448, 0x46284631, 0xf884f000, 0xbdf82000, 0x20004601, 
    0x20304770, 0x7008493d, 0x70082080, 0x483bbf00, 0x21807800, 0x28004008, 0x4838d0f9, 0x21317800, 
    0x28004008, 0x2001d001, 0x20004770, 0xb500e7fc, 0x49322044, 0xb67271c8, 0xffe3f7ff, 0x2000b662, 
    0xb508bd00, 0x92004602, 0x492c2009, 0x466871c8, 0x71887880, 0x78404668, 0x46687148, 0x71087800, 
    0xf7ffb672, 0x4603ffce, 0x4618b662, 0xb578bd08, 0x460e4605, 0x48209500, 0x78004448, 0x71c8491f, 
    0x4448481d, 0x28066800, 0x2807d002, 0xe001d104, 0xe0032304, 0xe0012308, 0xbf002304, 0x2400bf00, 
    0x4668e025, 0x49157880, 0x46687188, 0x71487840, 0x78004668, 0x78d07108, 0x789072c8, 0x78507288, 
    0x78107248, 0x2b087208, 0x79d0d107, 0x799073c8, 0x79507388, 0x79107348, 0x98007308, 0x900018c0, 
    0xb67218d2, 0xff8df7ff, 0x18e0b662, 0x42b4b284, 0x2000d3d7, 0x0000bd78, 0x00000004, 0x40020000, 
    0x4602b530, 0xd1072901, 0x4b174610, 0x4b178043, 0x23208043, 0xbf007003, 0xd1092902, 0x4b144610, 
    0x4b1481c3, 0x880381c3, 0x005b085b, 0xbf008003, 0xd10d2903, 0x4b104610, 0x4b106043, 0x68046083, 
    0x25806803, 0x252043ab, 0x401c432b, 0xbf006004, 0xd1062904, 0x23004610, 0x02242411, 0x60231904, 
    0x2000bf00, 0x0000bd30, 0x000020c5, 0x000028d9, 0x0000c520, 0x0000d928, 0xd928c520, 0x0000ffff, 
    0x00000000, 0x00000006, 
};

 const target_flash_t flash = {
    0x20000021, // Init
    0x2000003D, // UnInit
    0x2000006F, // EraseChip
    0x20000083, // EraseSector
    0x200000AF, // ProgramPage

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


