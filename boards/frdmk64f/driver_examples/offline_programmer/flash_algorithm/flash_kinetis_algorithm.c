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
    0x47700a80, 0x485e495f, 0x60084449, 0x4b5e2000, 0x615a2201, 0x46106118, 0xd0fd07c1, 0x2002495a,
    0x619a3940, 0x70086288, 0x47702000, 0x47702000, 0x4c56b5f8, 0x444c2032, 0x25004621, 0xc461260f,
    0x48523114, 0x44484f52, 0x91003c0c, 0x696047b8, 0xd10d2800, 0xc4612034, 0x4448484a, 0x60206800,
    0x3c0c484a, 0x99004448, 0x696047b8, 0xd0002800, 0xbdf82001, 0x4d45b5f8, 0x20320a84, 0xc511444d,
    0x310c4629, 0x4e424841, 0x460f602c, 0x3d084448, 0x696847b0, 0xd10e2800, 0xc5112034, 0x602c4839,
    0x68004448, 0x48396068, 0x44484639, 0x47b03d08, 0x28006968, 0x2001d000, 0xb5f7bdf8, 0x9802b082,
    0x0a804f32, 0x9001444f, 0x30144638, 0x90004614, 0x28009802, 0xcc03d10e, 0x18406862, 0x18896821,
    0x68a11840, 0x68e11840, 0x69211840, 0x42401840, 0x3c086160, 0x26009d01, 0xc7212032, 0x4823603d,
    0x44484a23, 0x99003f08, 0x69784790, 0xd11e2800, 0xc7212032, 0x481d603d, 0x44484a1d, 0x99003f08,
    0x69784790, 0xd1122800, 0x60382033, 0x60bc9802, 0x20406078, 0x481360f8, 0x44484a15, 0x61386800,
    0x99004812, 0x47904448, 0x28006978, 0x2001d002, 0xbdf0b005, 0xd0cf1c76, 0x60382038, 0x60bc9802,
    0x20406078, 0x480760f8, 0x44484a09, 0x61386800, 0x99004806, 0x47904448, 0x28006978, 0xe7e7d1a8,
    0x00003a98, 0x00000004, 0x40048040, 0x00000008, 0x0f001ff1, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

  target_flash_t flash_main = {
    0x10000025, // Init
    0x1000004d, // UnInit
    0x10000051, // EraseChip
    0x10000095, // EraseSector
    0x100000db, // ProgramPage

    // breakpoint = RAM start + 1
    // RSB : base address is address of Execution Region PrgData in map file
    //       to access global/static data
    // RSP : Initial stack pointer
    {
        0x10000001, // breakpoint instruction address
        0x100001B4,  // static base register value (image start + header + static base offset(static varibles))
        0x10000300  // initial stack pointer
    },

    0x10000700, // program_buffer, any valid RAM location with +512 bytes of headroom
    0x10000000, // algo_start, start of RAM
    sizeof(KIENTIS_FLM), // algo_size, size of array above
    KIENTIS_FLM,  // image, flash algo instruction array
    64        // ram_to_flash_bytes_to_be_written
};


