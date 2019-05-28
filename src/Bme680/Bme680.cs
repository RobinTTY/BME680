// Ported from https://github.com/BoschSensortec/BME680_driver/blob/master/bme680.c


using System;
using System.Buffers.Binary;
using System.Device.I2c;
using System.Threading.Tasks;
using Iot.Units;

namespace Bme680
{
    public class Bme680 : IDisposable
    {

        private readonly I2cDevice _i2cDevice;
        private bool _initialized;
        private CommunicationProtocol _protocol;
        private CalibrationData _calibrationData;
        private int _temperatureFine;
        
        // The BME680 uses two addresses: 0x76 (primary) and 0x77 (secondary)
        private const byte DefaultI2cAddress = 0x76;
        // The ChipId of the BME680
        private const byte _deviceId = 0x61;
        
        public Bme680(I2cDevice i2cDevice)
        {
            _i2cDevice = i2cDevice;
        }

        private enum CommunicationProtocol
        {
            I2C,
            Spi
        }

        // TODO: make private, maybe implement switches for communication device type
        /// <summary>
        /// Initializes the BMP680 sensor, making it ready for use.
        /// </summary>
        public void InitDevice()
        {
            byte readSignature;

            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    _i2cDevice.WriteByte((byte)Register.CHIPID);
                    readSignature = _i2cDevice.ReadByte();
                    break;

                case CommunicationProtocol.Spi:
                    throw new NotImplementedException();

                default:
                    throw new ArgumentOutOfRangeException();
            }

            if (readSignature != _deviceId)
                throw new Exception($"Device ID {readSignature} is not the same as expected {_deviceId}. Please check if you are using the right device.");

            _initialized = true;
            _calibrationData.ReadFromDevice(this);
        }

        /// <summary>
        /// Triggers a soft reset on the device which has the same effect as power-on reset.
        /// </summary>
        public void TriggerSoftReset()
        {
            switch (_protocol)
            {
                // TODO: do we need a delay after resetting? test read directly after reset
                case CommunicationProtocol.I2C:
                    _i2cDevice.Write(new[] { (byte)Register.RESET, (byte)0xB6 });
                    _initialized = false;
                    break;

                case CommunicationProtocol.Spi:
                    throw new ArgumentOutOfRangeException();

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        /// <summary>
        /// Sets the power mode to the given mode.
        /// </summary>
        /// <param name="powerMode"></param>
        public void SetPowerMode(PowerMode powerMode)
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
            status = (byte)(status & 0b1111_1100);
            status = (byte)(status & (byte)powerMode);

            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    _i2cDevice.Write(new[] { (byte)Register.CTRL_MEAS, status });
                    break;

                case CommunicationProtocol.Spi:
                    throw new NotImplementedException();

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        /// <summary>
        /// Reads the current power mode the device is running in.
        /// </summary>
        /// <returns></returns>
        public PowerMode ReadPowerMode()
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
            status = (byte)(status & 0b000_00011);

            // TODO: test this
            switch ((PowerMode)status)
            {
                case PowerMode.Sleep:
                    return PowerMode.Sleep;

                case PowerMode.Forced:
                    return PowerMode.Forced;

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        /// <summary>
        /// Sets the temperature sampling to the given value.
        /// </summary>
        /// <param name="sampling"></param>
        public void SetTemperatureSampling(Sampling sampling)
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
            status = (byte)(status & 0b0001_1111);
            status = (byte)(status | (byte)sampling << 5);

            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    _i2cDevice.Write(new[] { (byte)Register.CTRL_MEAS, status });
                    break;

                case CommunicationProtocol.Spi:
                    throw new NotImplementedException();

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        /// <summary>
        /// Get the sampling rate for temperature measurements.
        /// </summary>
        /// <returns></returns>
        public Sampling ReadTemperatureSampling()
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
            status = (byte)((status & 0b1110_0000) >> 5);
            return ByteToSampling(status);
        }

        /// <summary>
        /// Sets the pressure sampling to the given value.
        /// </summary>
        /// <param name="sampling"></param>
        public void SetPressureSampling(Sampling sampling)
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
            status = (byte)(status & 0b1110_0011);
            status = (byte)(status | (byte)sampling << 5);

            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    _i2cDevice.Write(new[] { (byte)Register.CTRL_MEAS, status });
                    break;

                case CommunicationProtocol.Spi:
                    throw new NotImplementedException();

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        /// <summary>
        /// Get the sampling rate for pressure measurements
        /// </summary>
        /// <returns></returns>
        public Sampling ReadPressureSampling()
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
            status = (byte)((status & 0b1110_0000) >> 5);
            return ByteToSampling(status);
        }

        /// <summary>
        /// Sets the humidity sampling to the given value
        /// </summary>
        /// <param name="sampling"></param>
        public void SetHumiditySampling(Sampling sampling)
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_HUM);
            status = (byte)(status & 0b1111_1000);
            status = (byte)(status | (byte)sampling << 5);

            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    _i2cDevice.Write(new[] { (byte)Register.CTRL_HUM, status });
                    break;

                case CommunicationProtocol.Spi:
                    throw new NotImplementedException();

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        /// <summary>
        /// Get the sampling rate for humidity measurements
        /// </summary>
        /// <returns></returns>
        public Sampling ReadHumiditySampling()
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_HUM);
            status = (byte)(status & 0b0000_0111);
            return ByteToSampling(status);
        }

        private Sampling ByteToSampling(byte value)
        {
            // Values >=5 equals X16
            if (value >= 5)
                return Sampling.X16;

            return (Sampling)value;
        }

        /// <summary>
        /// Sets the IIR filter to the given coefficient. The IIR filter affects temperature and pressure
        /// measurements but not humidity and gas measurements.
        /// </summary>
        /// <param name="coefficient"></param>
        public void SetFilter(FilterCoefficient coefficient)
        {
            var filter = Read8BitsFromRegister((byte)Register.CONFIG);
            filter = (byte)(filter & 0b1110_0011);
            filter = (byte)(filter | (byte)coefficient << 5);

            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    _i2cDevice.Write(new[] { (byte)Register.CONFIG, filter });
                    break;

                case CommunicationProtocol.Spi:
                    throw new NotImplementedException();

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        /// <summary>
        /// Get the currently set IIR filter coefficient.
        /// </summary>
        /// <returns></returns>
        public FilterCoefficient ReadFilterCoefficient()
        {
            var filter = Read8BitsFromRegister((byte)Register.CONFIG);
            filter = (byte)(filter & 0b0001_1100);
            return (FilterCoefficient)filter;
        }

        // TODO: measurements in c driver is done via read_field_data, datasheet 3.3.1 states that if
        // filter is enabled, resolution is 16 + (osrs_t - 1) bit, e.g. 18 bit when osrs_t is set to 3
        // this is not incorporated yet!
        /// <summary>
        /// Reads the temperature from the sensor.
        /// </summary>
        /// <returns>Temperature</returns>
        public async Task<Temperature> ReadTemperatureAsync()
        {
            if (!_initialized)
                InitDevice();

            // TODO: maybe don't set mode but have a wrapper which does all measurements based on
            // a mask, see how it's done in the official driver
            SetPowerMode(PowerMode.Forced);

            // TODO: we may need to wait here for a measurement to be performed after setting power mode
            // wait for measurement to be performed
            await Task.Delay(7); // TODO: how long do we need to wait? -> c driver -> get profile duration

            // TODO: check status registers if measurement was successful

            // Read 20 bit uncompensated temperature value from registers
            var t1 = Read16BitsFromRegister((byte)Register.TEMP, Endianness.BigEndian);
            var t2 = Read8BitsFromRegister((byte)Register.TEMP + 2 * sizeof(byte));

            // Combine the two values, t2 is only 4 bit
            var temp = (t1 << 4) + (t2 >> 4);

            return CompensateTemperature(temp);
        }

        /// <summary>
        ///  Reads the pressure from the sensor.
        /// </summary>
        /// <returns>Atmospheric pressure in Pa.</returns>
        public async Task<double> ReadPressureAsync()
        {
            if(!_initialized)
                InitDevice();

            SetPowerMode(PowerMode.Forced);

            if (_temperatureFine == int.MinValue)
                await ReadTemperatureAsync();

            // Read 20 bit uncompensated pressure value from registers
            var p1 = Read16BitsFromRegister((byte)Register.PRESS, Endianness.BigEndian);
            var p2 = Read8BitsFromRegister((byte)Register.PRESS + 2 * sizeof(byte));

            // Combine the two values, p2 is only 4 bit
            var press = (p1 << 4) + (p2 >> 4);

            // TODO: why divided by 256
            return CompensatePressure(press) / 256;
        }

        // TODO: really double?
        public async Task<double> ReadHumidityAsync()
        {
            if(!_initialized)
                InitDevice();

            SetPowerMode(PowerMode.Forced);

            if (_temperatureFine == int.MinValue)
                await ReadTemperatureAsync();

            // Read 16 bit uncompensated humidity value from registers
            var hum = Read16BitsFromRegister((byte)Register.HUM, Endianness.BigEndian);

            return CompensateHumidity(hum);
        }

        // TODO: Check if this summary applies
        /// <summary>
        ///  Returns the temperature. Resolution is 0.01 DegC. Output value of “5123” equals 51.23 degrees celsius.
        /// </summary>
        /// <param name="adcTemperature">The temperature value read from the device.</param>
        /// <returns>Temperature</returns>
        private Temperature CompensateTemperature(int adcTemperature)
        {
            var var1 = (adcTemperature / 16384.0 - _calibrationData.ParT1 / 1024.0) * _calibrationData.ParT2;
            var var2 = (adcTemperature / 131072.0 - _calibrationData.ParT1 / 8192.0);
            var2 = var2 * var2 * (_calibrationData.ParT3 * 16.0);

            // TODO:
            _temperatureFine = (int) (var1 + var2);

            var temperature = (var1 + var2) / 5120.0;

            return Temperature.FromCelsius(0);
        }

        // TODO: Check if this summary applies
        /// <summary>
        ///  Returns the pressure in Pa, in Q24.8 format (24 integer bits and 8 fractional bits).
        ///  Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
        /// </summary>
        /// <param name="adcPressure">The pressure value read from the device.</param>
        /// <returns>Pressure in hPa</returns>
        private double CompensatePressure(int adcPressure)
        {
            var var1 = _temperatureFine / 2.0 - 64000.0;
            var var2 = var1 * var1 * (_calibrationData.ParP6 / 131072.0);
            var2 += var1 * (_calibrationData.ParP5 * 2.0);
            var2 = var2 / 4.0 + _calibrationData.ParP4 * 65536.0;
            var1 = _calibrationData.ParP3 * var1 * var1 / 16384.0 + _calibrationData.ParP2 * var1 / 524288.0;
            var1 = (1.0 + var1 / 32768.0) * _calibrationData.ParP1;
            var pressure = 1048576.0 - adcPressure;

            if (var1 == 0)
                return 0;

            pressure = (pressure - var2 / 4096.0) * 6250.0 / var1;
            var1 = _calibrationData.ParP9 * pressure * pressure / 2147483648.0;
            var2 = pressure * (_calibrationData.ParP8 / 327768.0);
            // TODO: use Math.Pow for these after successful test
            var var3 = Math.Pow(pressure / 256.0, 3) * (_calibrationData.ParP10 / 131072.0);
            pressure += (var1 + var2 + var3 + _calibrationData.ParP7 * 128.0) / 16.0;

            return pressure;
        }

        private double CompensateHumidity(int adcHumidity)
        {
            // TODO:
        }

        internal byte Read8BitsFromRegister(byte register)
        {
            byte value;

            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    _i2cDevice.WriteByte(register);
                    value = _i2cDevice.ReadByte();
                    return value;

                case CommunicationProtocol.Spi:
                    throw new NotImplementedException();

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        internal ushort Read16BitsFromRegister(byte register, Endianness mode = Endianness.LittleEndian)
        {
            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    Span<byte> bytes = stackalloc byte[2];

                    _i2cDevice.WriteByte(register);
                    _i2cDevice.Read(bytes);

                    switch (mode)
                    {
                        case Endianness.LittleEndian:
                            return BinaryPrimitives.ReadUInt16LittleEndian(bytes);
                        case Endianness.BigEndian:
                            return BinaryPrimitives.ReadUInt16BigEndian(bytes);
                        default:
                            throw new ArgumentOutOfRangeException(nameof(mode), mode, null);
                    }


                case CommunicationProtocol.Spi:
                    throw new NotImplementedException();

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        internal enum Endianness
        {
            LittleEndian,
            BigEndian
        }

        public void Dispose()
        {
        }
    }
}
