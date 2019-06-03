// Ported from https://github.com/BoschSensortec/BME680_driver/blob/master/bme680.c
// TODO: create example, create Readme.md
// TODO: clarify how much API surface should exist, should user have full control or just set profiles and take measurements

using System;
using System.Buffers.Binary;
using System.Device.I2c;
using System.Device.Spi;
using System.Threading.Tasks;
using Iot.Units;

namespace Bme680
{
    public class Bme680 : IDisposable
    {

        private I2cDevice _i2cDevice;
        private SpiDevice _spiDevice;
        private CommunicationProtocol _protocol;

        private readonly CalibrationData _calibrationData;
        private int _temperatureFine;
        private bool _initialized;
        
        // The BME680 uses two addresses: 0x76 (primary) and 0x77 (secondary)
        private const byte DefaultI2cAddress = 0x76;
        // The ChipId of the BME680
        private const byte DeviceId = 0x61;

        /// <summary>
        /// Gets or sets whether the heater is enabled.
        /// </summary>
        public bool HeaterIsEnabled
        {
            get
            {
                var heaterStatus = Read8BitsFromRegister((byte)Register.CTRL_GAS_0);
                heaterStatus = (byte)((heaterStatus & (byte)Mask.HEAT_OFF) >> 3);
                return Convert.ToBoolean(heaterStatus);
            }
            set
            {
                var heaterStatus = Read8BitsFromRegister((byte)Register.CTRL_GAS_0);
                heaterStatus = (byte)(heaterStatus & ((byte)Mask.HEAT_OFF ^ (byte)Mask.CLR));
                heaterStatus = (byte)(heaterStatus | Convert.ToUInt32(value) << 4);

                Write8BitsToRegister((byte)Register.CTRL_GAS_0, heaterStatus);
            }
        }

        /// <summary>
        /// Gets or sets whether gas conversions are enabled.
        /// </summary>
        public bool GasConversionIsEnabled
        {
            get
            {
                var gasConversion = Read8BitsFromRegister((byte)Register.CTRL_GAS_1);
                gasConversion = (byte)((gasConversion & (byte)Mask.RUN_GAS) >> 4);
                return Convert.ToBoolean(gasConversion);
            }
            set
            {
                var gasConversion = Read8BitsFromRegister((byte)Register.CTRL_GAS_1);
                gasConversion = (byte)(gasConversion & ((byte)Mask.RUN_GAS ^ (byte)Mask.CLR));
                gasConversion = (byte)(gasConversion | Convert.ToUInt32(value) << 4);

                Write8BitsToRegister((byte)Register.CTRL_GAS_1, gasConversion);
            }
        }

        /// <summary>
        /// Indicates whether new data is available.
        /// </summary>
        public bool NewDataIsAvailable
        {
            get
            {
                var newData = Read8BitsFromRegister((byte)Register.MEAS_STATUS_0);
                newData = (byte)(newData >> 7);
                return Convert.ToBoolean(newData);
            }
        }

        /// <summary>
        /// Indicates whether a gas measurement is in process.
        /// </summary>
        public bool GasMeasurementInProcess
        {
            get
            {
                var gasMeasInProcess = Read8BitsFromRegister((byte)Register.MEAS_STATUS_0);
                gasMeasInProcess = (byte)((gasMeasInProcess & (byte)Mask.GAS_MEASURING) >> 6);
                return Convert.ToBoolean(gasMeasInProcess);
            }
        }

        /// <summary>
        /// Indicates whether a measurement is in process.
        /// </summary>
        public bool MeasurementInProcess
        {
            get
            {
                var measInProcess = Read8BitsFromRegister((byte)Register.MEAS_STATUS_0);
                measInProcess = (byte)((measInProcess & (byte)Mask.MEASURING) >> 5);
                return Convert.ToBoolean(measInProcess);
            }
        }

        /// <summary>
        /// Indicates whether a real gas conversion was performed (i.e. not a dummy one).
        /// </summary>
        public bool GasMeasurementIsValid
        {
            get
            {
                var gasMeasValid = Read8BitsFromRegister((byte)Register.GAS_R_LSB);
                gasMeasValid = (byte)((gasMeasValid & (byte)Mask.GAS_VALID) >> 5);
                return Convert.ToBoolean(gasMeasValid);
            }
        }

        /// <summary>
        /// Indicates whether the target heater temperature was reached.
        /// </summary>
        public bool HeaterIsStable
        {
            get
            {
                var heaterStable = Read8BitsFromRegister((byte)Register.GAS_R_LSB);
                heaterStable = (byte)((heaterStable & (byte)Mask.HEAT_STAB) >> 4);
                return Convert.ToBoolean(heaterStable);
            }
        }

        /// <summary>
        /// Gets or sets the temperature sampling.
        /// </summary>
        public Sampling TemperatureSampling
        {
            get
            {
                var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
                status = (byte)((status & (byte)Mask.TEMPERATURE_SAMPLING) >> 5);
                return ByteToSampling(status);
            }
            set
            {
                var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
                status = (byte)(status & ((byte)Mask.TEMPERATURE_SAMPLING ^ (byte)Mask.CLR));
                status = (byte)(status | (byte)value << 5);

                Write8BitsToRegister((byte)Register.CTRL_MEAS, status);
            }
        }

        /// <summary>
        /// Gets or sets the humidity sampling.
        /// </summary>
        public Sampling HumiditySampling
        {
            get
            {
                var status = Read8BitsFromRegister((byte)Register.CTRL_HUM);
                status = (byte)(status & (byte)Mask.HUMIDITY_SAMPLING);
                return ByteToSampling(status);
            }
            set
            {
                var status = Read8BitsFromRegister((byte)Register.CTRL_HUM);
                status = (byte)(status & ((byte)Mask.HUMIDITY_SAMPLING ^ (byte)Mask.CLR));
                status = (byte)(status | (byte)value);

                Write8BitsToRegister((byte)Register.CTRL_HUM, status);
            }
        }

        /// <summary>
        /// Gets or sets the IIR filter to the given coefficient. The IIR filter affects temperature and
        /// pressure measurements but not humidity and gas measurements. An IIR filter can suppress
        /// disturbances (e.g. slamming of a door or wind blowing in the sensor)
        /// </summary>
        public FilterCoefficient FilterCoefficient
        {
            get
            {
                var filter = Read8BitsFromRegister((byte)Register.CONFIG);
                filter = (byte)((filter & (byte)Mask.FILTER_COEFFICIENT) >> 2);
                return (FilterCoefficient)filter;
            }
            set
            {
                var filter = Read8BitsFromRegister((byte)Register.CONFIG);
                filter = (byte)(filter & ((byte)Mask.FILTER_COEFFICIENT ^ (byte)Mask.CLR));
                filter = (byte)(filter | (byte)value << 2);

                Write8BitsToRegister((byte)Register.CONFIG, filter);
            }
        }

        /// <summary>
        /// Gets or sets the pressure sampling.
        /// </summary>
        public Sampling PressureSampling
        {
            get
            {
                var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
                status = (byte)((status & (byte)Mask.PRESSURE_SAMPLING) >> 2);
                return ByteToSampling(status);
            }
            set
            {
                var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
                status = (byte)(status & ((byte)Mask.PRESSURE_SAMPLING ^ (byte)Mask.CLR));
                status = (byte)(status | (byte)value << 2);

                Write8BitsToRegister((byte)Register.CTRL_MEAS, status);
            }
        }

        private Sampling ByteToSampling(byte value)
        {
            // Values >=5 equals X16
            if (value >= 5)
                return Sampling.X16;

            return (Sampling)value;
        }

        public Bme680(I2cDevice i2cDevice)
        {
            _i2cDevice = i2cDevice;
            _calibrationData = new CalibrationData();
            _protocol = CommunicationProtocol.I2C;
        }

        private Bme680(SpiDevice spiDevice)
        {
            // not fully implemented yet, translation of memory addresses and memory page access missing
            throw new NotImplementedException();

            _spiDevice = spiDevice;
            _calibrationData = new CalibrationData();
            _protocol = CommunicationProtocol.Spi;
        }

        public enum CommunicationProtocol
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
            var readSignature = Read8BitsFromRegister((byte)Register.CHIP_ID);

            if (readSignature != DeviceId)
                throw new Exception($"Device ID {readSignature} is not the same as expected {DeviceId}. Please check if you are using the right device.");

            _initialized = true;
            _calibrationData.ReadFromDevice(this);
        }

        /// <summary>
        /// Triggers a soft reset on the device which has the same effect as power-on reset.
        /// </summary>
        public void TriggerSoftReset()
        {
            // TODO: do we need a delay after resetting? test read directly after reset
            Write8BitsToRegister((byte)Register.RESET, 0xB6);
            _initialized = false;
        }

        /// <summary>
        /// Sets the power mode to the given mode.
        /// </summary>
        /// <param name="powerMode"></param>
        public void SetPowerMode(PowerMode powerMode)
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
            status = (byte)(status & ((byte)Mask.PWR_MODE ^ (byte)Mask.CLR));
            status = (byte)(status & (byte)powerMode);

            Write8BitsToRegister((byte)Register.CTRL_MEAS, status);
        }

        /// <summary>
        /// Reads the current power mode the device is running in.
        /// </summary>
        /// <returns></returns>
        public PowerMode ReadPowerMode()
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
            status = (byte)(status & (byte)Mask.PWR_MODE);

            return (PowerMode)status;
        }

        /// <summary>
        /// Sets a gas-sensor-heater profile.
        /// </summary>
        /// <param name="profile">The chosen heater profile. Ranging from 0-9.</param>
        /// <param name="temperature">The desired temperature in °C. Ranging from 0 to 400</param>
        /// <param name="duration">The desired heating duration in ms. Ranging from 0-4032</param>
        /// <returns></returns>
        public async Task SetGasConfig(HeaterProfile profile, ushort temperature, ushort duration)
        {
            if (ReadPowerMode() != PowerMode.Forced)
                SetPowerMode(PowerMode.Forced);

            // read ambient temperature for resistance calculation
            var ambTemp = await ReadTemperatureAsync();
            var heaterResistance = CalculateHeaterResistance(temperature, (int)ambTemp.Celsius);
            var heaterDuration = CalculateHeaterDuration(duration);

            Write8BitsToRegister((byte)((byte)Register.RES_HEAT0 + profile), heaterResistance);
            Write8BitsToRegister((byte)((byte)Register.GAS_WAIT0 + profile), heaterDuration);
        }

        /// <summary>
        /// Gets the specified gas-sensor-heater profile.
        /// </summary>
        /// <param name="profile">The chosen heater profile.</param>
        /// <returns>The configuration of the chosen set-point or null if invalid set-point was chosen.</returns>
        public async Task<GasConfiguration> GetGasConfig(HeaterProfile profile)
        {
            if (ReadPowerMode() != PowerMode.Forced)
                SetPowerMode(PowerMode.Forced);

            // need to be converted?!
            var heaterTemp = Read8BitsFromRegister((byte)((byte)Register.RES_HEAT0 + profile));
            var heaterDuration = Read8BitsFromRegister((byte)((byte)Register.GAS_WAIT0 + profile));

            // read ambient temperature for resistance calculation
            var ambTemp = await ReadTemperatureAsync();
            heaterTemp = CalculateHeaterResistance(heaterTemp, (int)ambTemp.Celsius);
            heaterDuration = CalculateHeaterDuration(heaterDuration);

            return new GasConfiguration(heaterTemp, heaterDuration);
        }

        /// <summary>
        /// Selects the heater profile to be used for the next measurement.
        /// </summary>
        /// <param name="profile"></param>
        public void SelectHeaterProfile(HeaterProfile profile)
        {
            Write8BitsToRegister((byte)Register.CTRL_GAS_1, (byte)profile);
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
            await Task.Delay(7); // TODO: how long do we need to wait? -> c driver -> get profile duration -> dependent on oversampling

            // TODO: check status registers if measurement was successful

            // Read 20 bit uncompensated temperature value from registers
            var t1 = Read16BitsFromRegister((byte)Register.TEMP, Endianness.BigEndian);
            var t2 = Read8BitsFromRegister((byte)Register.TEMP + 2 * sizeof(byte));

            // Combine the two values, t2 is only 4 bit
            var temp = (t1 << 4) + (t2 >> 4);

            return CalculateTemperature(temp);
        }

        /// <summary>
        ///  Reads the pressure from the sensor.
        /// </summary>
        /// <returns>Atmospheric pressure in Pa.</returns>
        public async Task<double> ReadPressureAsync()
        {
            if (!_initialized)
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
            return CalculatePressure(press) / 256;
        }

        /// <summary>
        /// Reads the humidity from the sensor.
        /// </summary>
        /// <returns>Humidity in percent.</returns>
        public async Task<double> ReadHumidityAsync()
        {
            if (!_initialized)
                InitDevice();

            SetPowerMode(PowerMode.Forced);

            if (_temperatureFine == int.MinValue)
                await ReadTemperatureAsync();

            // Read 16 bit uncompensated humidity value from registers
            var hum = Read16BitsFromRegister((byte)Register.HUM, Endianness.BigEndian);

            return CalculateHumidity(hum);
        }

        // TODO: maybe name method differently
        /// <summary>
        /// Reads the gas resistance from the sensor.
        /// </summary>
        /// <returns>Gas resistance in Ohm.</returns>
        public async Task<double> ReadGasResistanceAsync()
        {
            if (!_initialized)
                InitDevice();

            SetPowerMode(PowerMode.Forced);

            // TODO: HEAT HERE?!
            // Read 10 bit gas resistance value from registers
            var g1 = Read8BitsFromRegister((byte)Register.GAS_RES);
            var g2 = Read8BitsFromRegister((byte)Register.GAS_RES + sizeof(byte));
            var gasRange = Read8BitsFromRegister((byte)Register.GAS_RANGE);

            var gasResistance = (g1 << 2) + (g2 >> 6);
            gasRange &= (byte)Mask.GAS_RANGE;

            return CalculateGasResistance(gasResistance, gasRange);
        }

        // TODO: check input variable type
        // TODO: Check if this summary applies
        /// <summary>
        ///  Returns the temperature. Resolution is 0.01 DegC. Output value of “5123” equals 51.23 degrees celsius.
        /// </summary>
        /// <param name="adcTemperature">The temperature value read from the device.</param>
        /// <returns>Temperature</returns>
        private Temperature CalculateTemperature(int adcTemperature)
        {
            var var1 = (adcTemperature / 16384.0 - _calibrationData.ParT1 / 1024.0) * _calibrationData.ParT2;
            var var2 = adcTemperature / 131072.0 - _calibrationData.ParT1 / 8192.0;
            var2 = var2 * var2 * (_calibrationData.ParT3 * 16.0);

            // TODO:
            _temperatureFine = (int)(var1 + var2);

            var temperature = (var1 + var2) / 5120.0;

            return Temperature.FromCelsius(0);
        }

        // TODO: check input variable type
        // TODO: Check if this summary applies
        /// <summary>
        ///  Returns the pressure in Pa, in Q24.8 format (24 integer bits and 8 fractional bits).
        ///  Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
        /// </summary>
        /// <param name="adcPressure">The pressure value read from the device.</param>
        /// <returns>Pressure in hPa</returns>
        private double CalculatePressure(int adcPressure)
        {
            var var1 = _temperatureFine / 2.0 - 64000.0;
            var var2 = Math.Pow(var1, 2) * (_calibrationData.ParP6 / 131072.0);
            var2 += var1 * _calibrationData.ParP5 * 2.0;
            var2 = var2 / 4.0 + _calibrationData.ParP4 * 65536.0;
            var1 = (_calibrationData.ParP3 * Math.Pow(var1, 2) / 16384.0 + _calibrationData.ParP2 * var1) / 524288.0;
            var1 = (1.0 + var1 / 32768.0) * _calibrationData.ParP1;
            var pressure = 1048576.0 - adcPressure;

            if (var1 == 0)
                return 0;

            pressure = (pressure - var2 / 4096.0) * 6250.0 / var1;
            var1 = _calibrationData.ParP9 * Math.Pow(pressure, 2) / 2147483648.0;
            var2 = pressure * (_calibrationData.ParP8 / 32768.0);
            var var3 = Math.Pow(pressure / 256.0, 3) * (_calibrationData.ParP10 / 131072.0);
            pressure += (var1 + var2 + var3 + _calibrationData.ParP7 * 128.0) / 16.0;

            return pressure;
        }

        // TODO: check input variable type
        private double CalculateHumidity(int adcHumidity)
        {
            var tempComp = _temperatureFine / 5120.0;
            var var1 = adcHumidity - (_calibrationData.ParH1 * 16.0 + _calibrationData.ParH3 / 2.0 * tempComp);
            var var2 = var1 * (float)(_calibrationData.ParH2 / 262144.0 * (1.0 + _calibrationData.ParH4 / 16384.0 * tempComp + _calibrationData.ParH5 / 1048576.0 * Math.Pow(tempComp, 2)));
            var var3 = _calibrationData.ParH6 / 16384.0;
            var var4 = _calibrationData.ParH7 / 2097152.0;
            var humidity = var2 + (var3 + var4 * tempComp) * var2 * var2;

            // limit possible value range
            if (humidity > 100.0)
                humidity = 100.0;
            else if (humidity < 0.0)
                humidity = 0.0;

            return humidity;
        }

        // TODO: check input variable type
        private double CalculateGasResistance(int adcGasRes, byte gasRange)
        {
            var k1Lookup = new[] { 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0 };
            var k2Lookup = new[] { 0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

            var var1 = 1340.0 + 5.0 * _calibrationData.RangeSwErr;
            var var2 = var1 * (1.0 + k1Lookup[gasRange] / 100.0);
            var var3 = 1.0 + k2Lookup[gasRange] / 100.0;
            var gasResistance = 1.0 / (var3 * 0.000000125 * (1 << gasRange) * ((adcGasRes - 512.0) / var2 + 1.0));

            return gasResistance;
        }

        // TODO: check who needs to call this method
        // TODO: check input variable type
        // cast to byte correct? direct test with C++ Code returns same results so probably fine
        private byte CalculateHeaterResistance(int setTemp, int ambientTemp)
        {
            // limit maximum temperature to 400°C
            if (setTemp > 400)
                setTemp = 400;

            var var1 = _calibrationData.ParGh1 / 16.0 + 49.0;
            var var2 = _calibrationData.ParGh2 / 32768.0 * 0.0005 + 0.00235;
            var var3 = _calibrationData.ParGh3 / 1024.0;
            var var4 = var1 * (1.0 + var2 * setTemp);
            var var5 = var4 + var3 * ambientTemp;
            var heaterResistance = (byte)(3.4 * (var5 * (4.0 / (4.0 + _calibrationData.ResHeatRange)) * (1.0 / (1.0 + _calibrationData.ResHeatVal * 0.002)) - 25));

            return heaterResistance;
        }

        // TODO: check who needs to call this method
        private byte CalculateHeaterDuration(ushort duration)
        {
            byte factor = 0;
            byte durationValue;

            // check if value exceeds maximum duration
            if (duration > 0xFC0)
                durationValue = 0xFF;
            else
            {
                while (duration > 0x3F)
                {
                    duration = (ushort)(duration >> 2);
                    factor += 1;
                }

                durationValue = (byte)(duration + factor * 64);
            }

            return durationValue;
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
                    _spiDevice.WriteByte(register);
                    value = _spiDevice.ReadByte();
                    return value;

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        internal ushort Read16BitsFromRegister(byte register, Endianness mode = Endianness.LittleEndian)
        {
            Span<byte> bytes = stackalloc byte[2];

            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    _i2cDevice.WriteByte(register);
                    _i2cDevice.Read(bytes);
                    break;

                case CommunicationProtocol.Spi:
                    _spiDevice.WriteByte(register);
                    _spiDevice.Read(bytes);
                    break;

                default:
                    throw new ArgumentOutOfRangeException();
            }

            switch (mode)
            {
                case Endianness.LittleEndian:
                    return BinaryPrimitives.ReadUInt16LittleEndian(bytes);
                case Endianness.BigEndian:
                    return BinaryPrimitives.ReadUInt16BigEndian(bytes);
                default:
                    throw new ArgumentOutOfRangeException(nameof(mode), mode, null);
            }
        }

        private void Write8BitsToRegister(byte register, byte data)
        {
            switch (_protocol)
            {
                case CommunicationProtocol.I2C:
                    _i2cDevice.Write(new[] { register, data });
                    break;

                case CommunicationProtocol.Spi:
                    _spiDevice.Write(new[] { register, data });
                    break;

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
            if (_i2cDevice != null)
            {
                _i2cDevice.Dispose();
                _i2cDevice = null;
            }

            if (_spiDevice != null)
            {
                _spiDevice.Dispose();
                _spiDevice = null;
            }
        }
    }
}
