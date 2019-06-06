namespace Bme680
{
    // Adresses differ for SPI implementation but can be translated using mask, see bme680_get_regs in
    // the Bosch C driver: https://github.com/BoschSensortec/BME680_driver/blob/master/bme680.c
    internal enum Register : byte
    {
        PAR_T1 = 0xE9,      // ok - 2 byte
                            // 1 byte empty
        PAR_T2 = 0x8A,      // 2 byte Begin 1 off??
        PAR_T3 = 0x8C,      // 1 byte
                            // 1 byte empty (8E)
        PAR_P1 = 0x8E,      // 2 byte
        PAR_P2 = 0x90,      // 2 byte
        PAR_P3 = 0x92,      // 1 byte
                            // 1 byte empty (94)
        PAR_P4 = 0x94,      // 2 byte
        PAR_P5 = 0x96,      // 2 byte
        PAR_P6 = 0x99,      // 1 byte switched places with P7
                            // 2 byte empty  (9B/9C)
        PAR_P7 = 0x98,      // 1 byte switched places with P6
        PAR_P8 = 0x9C,      // 2 byte
        PAR_P9 = 0x9E,      // 2 byte
        PAR_P10 = 0xA0,     // ok - 1 byte
                            // 1 byte empty // 25 byte - end of region 1
        PAR_H1_LSB = 0xE2,  // ok - 2 byte switched with H1 start overlaps with H2 end
        PAR_H1_MSB = 0xE3,
        PAR_H2_LSB = 0xE2,  // ok - 2 byte CAUTION: SWAP OF LSB / MSB -> what to do? (LSB = 26, MSB = 25) maybe read 2 * 8 bit portions
        PAR_H2_MSB = 0xE1,
        PAR_H3 = 0xE4,      // ok - 1 byte
        PAR_H4 = 0xE5,      // ok - 1 byte
        PAR_H5 = 0xE6,      // ok - 1 byte
        PAR_H6 = 0xE7,      // ok - 1 byte
        PAR_H7 = 0xE8,      // ok - 1 byte
                            // 2 byte T1 addressed in this spot
        PAR_GH1 = 0xED,     // ok - 1 byte switched places with GH2
        PAR_GH2 = 0xEB,     // ok - 2 byte switched places with GH1
        PAR_GH3 = 0xEE,     // ok - 1 byte   // 16 byte - end of region 2

        CHIP_ID = 0xD0,
        CTRL_HUM = 0x72,
        CTRL_MEAS = 0x74,
        CONFIG = 0x75,
        RESET = 0xE0,

        PRESS = 0x1F,           // 20bit buff[0] = 1d | buff[1] = 1e | buff[2] = 1F
        TEMP = 0x22,            // 20bit buff[5] = 22
        HUM = 0x25,             // 20bit buff[8] = 25
        GAS_RES = 0x2A,         // 8bit buff[13] = 2A
        GAS_RANGE = 0x2B,       // mask!

        RES_HEAT_VAL = 0x00,
        RES_HEAT_RANGE = 0x02,
        RANGE_SW_ERR = 0x04,
        
        RES_HEAT0 = 0x5A,
        GAS_WAIT0 = 0x64,
        CTRL_GAS_0 = 0x70,
        CTRL_GAS_1 = 0x71,

        MEAS_STATUS_0 = 0x1D,
        GAS_R_LSB = 0x2B
    }
}
