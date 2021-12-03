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
    0xf240b580, 0xf2c00004, 0xf6420000, 0xf84961e0, 0xf2401000, 0xf2c52000, 0x21000000, 0x1080f8c0,
    0x1084f8c0, 0x1180f8c0, 0x71fbf647, 0xf6406001, 0x21ff6004, 0x0000f2c5, 0x01def2cc, 0xf04f6001,
    0x210240a0, 0xf2407001, 0xf2c00010, 0x44480000, 0xf874f000, 0xbf182800, 0xbd802001, 0x47702000,
    0xf240b580, 0xf2c00010, 0xf2460000, 0x4448636c, 0xf6c62100, 0xf44f3365, 0xf0002218, 0x2800f87f,
    0x2001bf18, 0xbf00bd80, 0xf020b580, 0xf2404170, 0xf2c00010, 0xf2460000, 0x4448636c, 0x3365f6c6,
    0x4200f44f, 0xf86af000, 0xbf182800, 0xbd802001, 0x4614b570, 0x0441460d, 0x4670f020, 0xf240d10d,
    0xf2c00010, 0xf2460000, 0x4448636c, 0xf6c64631, 0xf44f3365, 0xf0004200, 0xf240f851, 0xf2c00010,
    0xf5b50000, 0xbf987f00, 0x7500f44f, 0x46314448, 0x462b4622, 0xf86af000, 0xbf182800, 0xbd702001,
    0x460cb5b0, 0xf0204605, 0x46114070, 0xf0004622, 0x2800fa01, 0x4425bf08, 0xbdb04628, 0x460ab580,
    0x4170f020, 0x0010f240, 0x0000f2c0, 0xf0004448, 0x2800f875, 0x2001bf18, 0x0000bd80, 0x02f4f241,
    0x3200f2c1, 0x290068d1, 0x2360d00a, 0x78926283, 0xf2406849, 0xf2c0030c, 0xf8490300, 0x47082003,
    0x40baf240, 0x0000f2c0, 0x41c7f240, 0x0100f2c0, 0x44794478, 0xf0002284, 0xbf00f98d, 0x0c0cf240,
    0x0c00f2c0, 0xc00cf859, 0x0f02f1bc, 0xf244d104, 0xf2c11c3b, 0x47603c00, 0x1c00f241, 0x3c00f2c1,
    0xc000f8dc, 0x0f00f1bc, 0xf8dcd002, 0x4760c008, 0x406af240, 0x0000f2c0, 0x4177f240, 0x0100f2c0,
    0x44794478, 0xf0002295, 0xbf00f965, 0x0c0cf240, 0x0c00f2c0, 0xc00cf859, 0x0f02f1bc, 0xf244d104,
    0xf2c11c9d, 0x47603c00, 0x1c00f241, 0x3c00f2c1, 0xc000f8dc, 0x0f00f1bc, 0xf8dcd002, 0x4760c00c,
    0x401af240, 0x0000f2c0, 0x4127f240, 0x0100f2c0, 0x44794478, 0xf00022a5, 0xbf00f93d, 0x1300f241,
    0x3300f2c1, 0x2b00681b, 0x691bd001, 0xf2404718, 0xf2c030ec, 0xf2400000, 0xf2c031f9, 0x44780100,
    0x22ad4479, 0xf926f000, 0x0c0cf240, 0x0c00f2c0, 0xc00cf859, 0x0f02f1bc, 0xf244d104, 0xf2c12c7d,
    0x47603c00, 0x1c00f241, 0x3c00f2c1, 0xc000f8dc, 0x0f00f1bc, 0xf8dcd002, 0x4760c014, 0x309ef240,
    0x0000f2c0, 0x31abf240, 0x0100f2c0, 0x44794478, 0xf00022c2, 0xbf00f8ff, 0x1300f241, 0x3300f2c1,
    0x2b00681b, 0x699bd001, 0xf2404718, 0xf2c03070, 0xf2400000, 0xf2c0317d, 0x44780100, 0x22cb4479,
    0xf8e8f000, 0x1100f241, 0x3100f2c1, 0x29006809, 0x6a89d001, 0xf2404708, 0xf2c03044, 0xf2400000,
    0xf2c03151, 0x44780100, 0x22d54479, 0xf8d2f000, 0x1100f241, 0x3100f2c1, 0x29006809, 0x6ac9d001,
    0xf2404708, 0xf2c03018, 0xf2400000, 0xf2c03125, 0x44780100, 0x22dc4479, 0xf8bcf000, 0x1300f241,
    0x3300f2c1, 0x2b00681b, 0x6b1bd001, 0xf2404718, 0xf2c020ec, 0xf2400000, 0xf2c021f9, 0x44780100,
    0x22e34479, 0xf8a6f000, 0x1200f241, 0x3200f2c1, 0x2a006812, 0x6b52d001, 0xf2404710, 0xf2c020c0,
    0xf2400000, 0xf2c021cd, 0x44780100, 0x22ea4479, 0xf890f000, 0x1c00f241, 0x3c00f2c1, 0xc000f8dc,
    0x0f00f1bc, 0xf8dcd002, 0x4760c038, 0x208ef240, 0x0000f2c0, 0x219bf240, 0x0100f2c0, 0x44794478,
    0xf00022f1, 0xbf00f877, 0x1200f241, 0x3200f2c1, 0x2a006812, 0x6bd2d001, 0xf2404710, 0xf2c02060,
    0xf2400000, 0xf2c0216d, 0x44780100, 0x22f84479, 0xf860f000, 0x1200f241, 0x3200f2c1, 0x2a006812,
    0x6c12d001, 0xf2404710, 0xf2c02034, 0xf2400000, 0xf2c02141, 0x44780100, 0x22ff4479, 0xf84af000,
    0x1300f241, 0x3300f2c1, 0x2b00681b, 0x6c5bd001, 0xf2404718, 0xf2c02008, 0xf2400000, 0xf2c02115,
    0x44780100, 0xf44f4479, 0xf0007283, 0xbf00f833, 0x1300f241, 0x3300f2c1, 0x2b00681b, 0x6c9bd001,
    0xf2404718, 0xf2c010d8, 0xf2400000, 0xf2c011e5, 0x44780100, 0xf2404479, 0xf000120d, 0xbf00f81b,
    0x1c00f241, 0x3c00f2c1, 0xc000f8dc, 0x0f00f1bc, 0xf8dcd002, 0x4760c04c, 0x10a2f240, 0x0000f2c0,
    0x11aff240, 0x0100f2c0, 0x44794478, 0x728af44f, 0xf800f000, 0x4605b50e, 0x460e4614, 0xf000a013,
    0x4628f870, 0xf86df000, 0xf000a016, 0x4630f86a, 0xf867f000, 0xf000a015, 0x2100f864, 0x100bf88d,
    0xf10d210a, 0xf88d000a, 0xe008100a, 0xf2f1fb94, 0x4212fb01, 0xf4f1fb94, 0xf8003230, 0x2c002d01,
    0xf000dcf4, 0xf000f84e, 0x0000f841, 0x202a2a2a, 0x65737361, 0x6f697472, 0x6166206e, 0x64656c69,
    0x0000203a, 0x6966202c, 0x0020656c, 0x696c202c, 0x0020656e, 0x0301ea40, 0x079bb510, 0x2a04d10f,
    0xc810d30d, 0x1f12c908, 0xd0f8429c, 0xba19ba20, 0xd9014288, 0xbd102001, 0x30fff04f, 0xb11abd10,
    0xd00307d3, 0xe0071c52, 0xbd102000, 0x3b01f810, 0x4b01f811, 0xd1071b1b, 0x3b01f810, 0x4b01f811,
    0xd1011b1b, 0xd1f11e92, 0xbd104618, 0x2000b510, 0xf81ef000, 0x8000f3af, 0x4010e8bd, 0xf0002001,
    0xb510b811, 0xe0024604, 0xf0001c64, 0x7820f804, 0xd1f92800, 0xb508bd10, 0xf88d4669, 0x20030000,
    0xbd08beab, 0x20184901, 0xe7febeab, 0x00020026, 0xf000b510, 0xe8bdf80b, 0xf0004010, 0x4770b801,
    0xd0012800, 0xbfeef7ff, 0x00004770, 0x2100b510, 0xf000a002, 0x2001f813, 0x0000bd10, 0x41474953,
    0x3a545242, 0x6e624120, 0x616d726f, 0x6574206c, 0x6e696d72, 0x6f697461, 0x0000006e, 0x4605b570,
    0x200a460c, 0x1c6de000, 0xffc5f7ff, 0x7828b135, 0xd1f82800, 0x1c64e002, 0xffbdf7ff, 0x7820b114,
    0xd1f82800, 0x4070e8bd, 0xf7ff200a, 0x4c46bfb4, 0x5f485341, 0x5f495041, 0x45455254, 0x70616900,
    0x73662f31, 0x61695f6c, 0x632e3170, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

 const target_flash_t flash = {
    0x20000021, // Init
    0x2000007d, // UnInit
    0x20000081, // EraseChip
    0x200000a9, // EraseSector
    0x200000d1, // ProgramPage


    // breakpoint = RAM start + 1
    // RSB : base address is address of Execution Region PrgData in map file
    //       to access global/static data
    // RSP : Initial stack pointer
    {
        0x20000001,
        0x20000670,
        0x20001000
    },

    0x20001800, // program_buffer, any valid RAM location with +512 bytes of headroom
    0x20000000, // algo_start, start of RAM
    sizeof(KIENTIS_FLM), // algo_size, size of array above
    KIENTIS_FLM,  // image, flash algo instruction array
    512        // ram_to_flash_bytes_to_be_written
};

