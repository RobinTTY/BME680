namespace Bme680
{
    // Addresses differ for SPI!
    internal enum Register : byte
    {
        PAR_T1 = 0xE9,  // far off
                        // 1 byte empty
        PAR_T2 = 0x8B,  // 2 byte Begin 1 off??
        PAR_T3 = 0x8D,  // 1 byte
                        // 1 byte empty (8E)
        PAR_P1 = 0x8F,  // 2 byte
        PAR_P2 = 0x91,  // 2 byte
        PAR_P3 = 0x93,  // 1 byte
                        // 1 byte empty (94)
        PAR_P4 = 0x95,  // 2 byte
        PAR_P5 = 0x97,  // 2 byte
        PAR_P6 = 0x9A,  // 1 byte switched places with P7
                        // 2 byte empty  (9B/9C)
        PAR_P7 = 0x99,  // 1 byte switched places with P6
        PAR_P8 = 0x9D,  // 2 byte
        PAR_P9 = 0x9F,  // 2 byte
        PAR_P10 = 0xA0, // 1 byte
                        // 1 byte empty // 25 byte - end of region 1

        PAR_H1 = 0xE2,  // 2 byte switched with H1 start overlaps with H2 end
        PAR_H2 = 0xE1,  // 2 byte CAUTION: SWAP OF LSB / MSB -> what to do? (LSB = 26, MSB = 25) maybe read 2 * 8 bit portions
        PAR_H3 = 0xE4,  // 1 byte
        PAR_H4 = 0xE5,  // 1 byte
        PAR_H5 = 0xE6,  // 1 byte
        PAR_H6 = 0xE7,  // 1 byte
        PAR_H7 = 0xE8,  // 1 byte
                        // 2 byte T1 addressed in this spot
        PAR_GH1 = 0xED, // 1 byte switched places with GH2
        PAR_GH2 = 0xEB, // 2 byte switched places with GH1
        PAR_GH3 = 0xEE, // 1 byte   // 16 byte - end of region 2

        CHIPID = 0xD0,
        CTRL_HUM = 0x72,
        CTRL_MEAS = 0x74,
        CONFIG = 0x75,
        RESET = 0xE0,

        PRESS = 0x1F,           // 20bit buff[0] = 1d | buff[1] = 1e | buff[2] = 1F
        TEMP = 0x22,            // 20bit buff[5] = 22
        HUM = 0x25,             // 20bit buff[8] = 25
        GAS_RES = 0x2A,         // 8bit buff[13] = 2A
        GAS_RANGE = 0x2B,       // mask!

        ADDR_RES_HEAT_VAL_ADDR = 0x00,
        ADDR_RES_HEAT_RANGE_ADDR = 0x02,
        ADDR_RANGE_SW_ERR_ADDR = 0x04
    }
}
