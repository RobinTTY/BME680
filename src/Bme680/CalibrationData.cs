using System;
using System.Collections.Generic;
using System.Text;

namespace Bme680
{
    // TODO: data types correct?
    internal class CalibrationData
    {
        public ushort ParH1 { get; set; }
        public ushort ParH2 { get; set; }
        public sbyte ParH3 { get; set; }
        public sbyte ParH4 { get; set; }
        public sbyte ParH5 { get; set; }
        public byte ParH6 { get; set; }
        public sbyte ParH7 { get; set; }
        public sbyte ParGh1 { get; set; }
        public short ParGh2 { get; set; }
        public sbyte ParGh3 { get; set; }
        public ushort ParT1 { get; set; }
        public short ParT2 { get; set; }
        public sbyte ParT3 { get; set; }
        public ushort ParP1 { get; set; }
        public short ParP2 { get; set; }
        public sbyte ParP3 { get; set; }
        public short ParP4 { get; set; }
        public short ParP5 { get; set; }
        public sbyte ParP6 { get; set; }
        public sbyte ParP7 { get; set; }
        public short ParP8 { get; set; }
        public short ParP9 { get; set; }
        public byte ParP10 { get; set; }

        internal void ReadFromDevice(Bme680 bme680)
        {
            // load humidity calibration data
            ParH1 = bme680.Read16BitsFromRegister((byte)Register.PAR_H1);
            ParH2 = bme680.Read16BitsFromRegister((byte)Register.PAR_H2);
            ParH3 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_H3);
            ParH4 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_H4);
            ParH5 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_H5);
            ParH6 = bme680.Read8BitsFromRegister((byte)Register.PAR_H6);
            ParH7 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_H7);

            // load gas calibration data
            ParGh1 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_GH1);
            ParGh2 = (short)bme680.Read16BitsFromRegister((byte)Register.PAR_GH2);
            ParGh3 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_GH3);

            // load temperature calibration data
            ParT1 = bme680.Read16BitsFromRegister((byte)Register.PAR_T1);
            ParT2 = (short)bme680.Read16BitsFromRegister((byte)Register.PAR_T2);
            ParT3 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_T3);

            // load pressure calibration data
            ParP1 = bme680.Read16BitsFromRegister((byte)Register.PAR_P1);
            ParP2 = (short)bme680.Read16BitsFromRegister((byte)Register.PAR_P2);
            ParP3 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_P3);
            ParP4 = (short)bme680.Read16BitsFromRegister((byte)Register.PAR_P4);
            ParP5 = (short)bme680.Read16BitsFromRegister((byte)Register.PAR_P5);
            ParP6 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_P6);
            ParP7 = (sbyte)bme680.Read8BitsFromRegister((byte)Register.PAR_P7);
            ParP8 = (short)bme680.Read16BitsFromRegister((byte)Register.PAR_P8);
            ParP9 = (short)bme680.Read16BitsFromRegister((byte)Register.PAR_P9);
            ParP10 = bme680.Read8BitsFromRegister((byte)Register.PAR_P10);
        }
    }

}
