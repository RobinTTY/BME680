// Ported from https://github.com/BoschSensortec/BME680_driver/blob/master/bme680.c

using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Device.I2c;
using System.Device.Spi;
using System.Linq;
using System.Threading.Tasks;

namespace Bme680Driver
{
    /// <summary>
    /// Temperature, humidity, pressure and gas resistance sensor Bme680.
    /// </summary>
    public class Bme680 : IDisposable
    {
        private static readonly byte[] OsToMeasCycles = { 0, 1, 2, 4, 8, 16 };
        private static readonly byte[] OsToSwitchCount = { 0, 1, 1, 1, 1, 1 };
        private static readonly double[] K1Lookup = { 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8, 0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0 };
        private static readonly double[] K2Lookup = { 0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

        private readonly List<HeaterProfileConfiguration> _heaterConfigs = new List<HeaterProfileConfiguration>();
        private readonly CalibrationData _calibrationData;
        private bool _gasConversionIsEnabled;
        private bool _heaterIsEnabled;
        private int _temperatureFine;

        private HeaterProfile _heaterProfile;
        private FilterCoefficient _filterCoefficient;
        private Sampling _temperatureSampling;
        private Sampling _humiditySampling;
        private Sampling _pressureSampling;

        // ReSharper disable once InconsistentNaming
        private I2cDevice _i2cDevice;
        private SpiDevice _spiDevice;
        private readonly CommunicationProtocol _protocol;
        
        // The ChipId of the BME680
        private const byte DeviceId = 0x61;

        /// <summary>
        /// The default I2c address of the Bme680.
        /// </summary>
        // ReSharper disable once UnusedMember.Global
        // ReSharper disable once InconsistentNaming
        public const byte DefaultI2cAddress = 0x76;

        /// <summary>
        /// The secondary I2c address of the Bme680.
        /// </summary>
        // ReSharper disable once UnusedMember.Global
        // ReSharper disable once InconsistentNaming
        public const byte SecondaryI2cAddress = 0x77;

        /// <summary>
        /// Temperature, humidity, pressure and gas resistance sensor Bme680.
        /// </summary>
        /// <param name="i2cDevice">The used I2c communication device.</param>
        // ReSharper disable once UnusedMember.Global
        // ReSharper disable once InconsistentNaming
        public Bme680(I2cDevice i2cDevice)
        {
            _i2cDevice = i2cDevice;
            _calibrationData = new CalibrationData();
            _protocol = CommunicationProtocol.I2C;

            // Check device signature which should be the same for all Bme680 sensors
            var readSignature = Read8BitsFromRegister((byte)Register.CHIP_ID);
            if (readSignature != DeviceId)
                throw new Exception($"Device ID {readSignature} is not the same as expected {DeviceId}. Please check if you are using the right device.");

            _calibrationData.ReadFromDevice(this);
            SetDefaultConfiguration();
        }

        /// <summary>
        /// Gets or sets whether the heater is enabled.
        /// </summary>
        public bool HeaterIsEnabled
        {
            get => _heaterIsEnabled;
            set
            {
                var heaterStatus = Read8BitsFromRegister((byte)Register.CTRL_GAS_0);
                heaterStatus = (byte)((heaterStatus & (byte)~Mask.HEAT_OFF) | Convert.ToByte(!value) << 3);

                Write8BitsToRegister((byte)Register.CTRL_GAS_0, heaterStatus);
                _heaterIsEnabled = value;
            }
        }

        /// <summary>
        /// Gets or sets whether gas conversions are enabled.
        /// </summary>
        public bool GasConversionIsEnabled
        {
            get => _gasConversionIsEnabled;
            set
            {
                var gasConversion = Read8BitsFromRegister((byte)Register.CTRL_GAS_1);
                gasConversion = (byte)((gasConversion & (byte)~Mask.RUN_GAS) | Convert.ToByte(value) << 4);

                Write8BitsToRegister((byte)Register.CTRL_GAS_1, gasConversion);
                _gasConversionIsEnabled = value;
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
        /// Gets or sets the heater profile to be used for the next measurement.
        /// </summary>
        public HeaterProfile CurrentHeaterProfile
        {
            get => _heaterProfile;
            set
            {
                if (_heaterConfigs.Exists(config => config.HeaterProfile == value))
                {
                    if (!Enum.IsDefined(typeof(HeaterProfile), value))
                        throw new ArgumentOutOfRangeException();

                    var heaterProfile = Read8BitsFromRegister((byte) Register.CTRL_GAS_1);
                    heaterProfile = (byte) ((heaterProfile & (byte) ~Mask.NB_CONV) | (byte) value);

                    Write8BitsToRegister((byte)Register.CTRL_GAS_1, heaterProfile);
                    _heaterProfile = value;
                }
            }
        }

        /// <summary>
        /// Gets or sets the IIR filter to the given coefficient.
        /// <para />
        /// The IIR filter affects temperature and pressure measurements but not humidity and gas measurements. 
        /// <para />
        /// An IIR filter can suppress disturbances (e.g. slamming of a door or wind blowing in the sensor).
        /// </summary>
        public FilterCoefficient FilterCoefficient
        {
            get => _filterCoefficient;
            set
            {
                if (!Enum.IsDefined(typeof(FilterCoefficient), value))
                    throw new ArgumentOutOfRangeException();

                var filter = Read8BitsFromRegister((byte)Register.CONFIG);
                filter = (byte)((filter & (byte)~Mask.FILTER_COEFFICIENT) | (byte)value << 2);

                Write8BitsToRegister((byte)Register.CONFIG, filter);
                _filterCoefficient = value;
            }
        }

        /// <summary>
        /// Gets or sets the temperature sampling.
        /// </summary>
        public Sampling TemperatureSampling
        {
            get => _temperatureSampling;
            set
            {
                if (!Enum.IsDefined(typeof(Sampling), value))
                    throw new ArgumentOutOfRangeException();

                var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
                status = (byte)((status & (byte)~Mask.TEMPERATURE_SAMPLING) | (byte)value << 5);

                Write8BitsToRegister((byte)Register.CTRL_MEAS, status);
                _temperatureSampling = value;
            }
        }

        /// <summary>
        /// Gets or sets the humidity sampling.
        /// </summary>
        public Sampling HumiditySampling
        {
            get => _humiditySampling;
            set
            {
                if (!Enum.IsDefined(typeof(Sampling), value))
                    throw new ArgumentOutOfRangeException();

                var status = Read8BitsFromRegister((byte)Register.CTRL_HUM);
                status = (byte)((status & (byte)~Mask.HUMIDITY_SAMPLING) | (byte)value);

                Write8BitsToRegister((byte)Register.CTRL_HUM, status);
                _humiditySampling = value;
            }
        }

        /// <summary>
        /// Gets or sets the pressure sampling.
        /// </summary>
        public Sampling PressureSampling
        {
            get => _pressureSampling;
            set
            {
                if (!Enum.IsDefined(typeof(Sampling), value))
                    throw new ArgumentOutOfRangeException();

                var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
                status = (byte)((status & (byte)~Mask.PRESSURE_SAMPLING) | (byte)value << 2);

                Write8BitsToRegister((byte)Register.CTRL_MEAS, status);
                _pressureSampling = value;
            }
        }

        private enum CommunicationProtocol
        {
            I2C,
            Spi
        }

        /// <summary>
        /// Triggers a soft reset which sets the device back to default settings.
        /// </summary>
        public void TriggerSoftReset()
        {
            Write8BitsToRegister((byte)Register.RESET, 0xB6);
            SetDefaultConfiguration();
        }

        /// <summary>
        /// Performs an asynchronous measurement by setting the sensor to forced mode, awaits the result.
        /// </summary>
        /// <returns><see cref="Bme680ReadResult"/> containing the measured values.</returns>
        public async Task<Bme680ReadResult> PerformMeasurementAsync()
        {
            var duration = GetProfileDuration(CurrentHeaterProfile);
            SetPowerMode(PowerMode.Forced);

            if (GasConversionIsEnabled)
                HeaterIsEnabled = false;

            await Task.Delay(duration);
            return ReadResultRegisters();
        }

        /// <summary>
        /// Performs a measurement by setting the sensor to forced mode, awaits the result.
        /// </summary>
        /// <returns><see cref="Bme680ReadResult"/> containing the measured values.</returns>
        public Bme680ReadResult PerformMeasurement()
        {
            var duration = GetProfileDuration(CurrentHeaterProfile);
            SetPowerMode(PowerMode.Forced);

            if (GasConversionIsEnabled)
                HeaterIsEnabled = false;

            Task.Delay(duration).Wait();
            return ReadResultRegisters();
        }

        /// <summary>
        /// Gets the required time in ms to perform a measurement with the given heater profile.
        /// The precision of this duration is within 1ms of the actual measurement time.
        /// </summary>
        /// <param name="profile">The heater profile.</param>
        /// <returns></returns>
        public int GetProfileDuration(HeaterProfile profile)
        {
            var measCycles = OsToMeasCycles[(int)TemperatureSampling];
            measCycles += OsToMeasCycles[(int)PressureSampling];
            measCycles += OsToMeasCycles[(int)HumiditySampling];

            var switchCount = OsToSwitchCount[(int)TemperatureSampling];
            switchCount += OsToSwitchCount[(int)PressureSampling];
            switchCount += OsToSwitchCount[(int)HumiditySampling];

            double measDuration = measCycles * 1963;
            measDuration += 477 * switchCount;      // TPH switching duration

            if (GasConversionIsEnabled)
                measDuration += 477 * 5;            // Gas measurement duration
            measDuration += 500;                    // get it to the closest whole number
            measDuration /= 1000.0;                 // convert to ms
            measDuration += 1;                      // wake up duration of 1ms

            if (GasConversionIsEnabled && _heaterConfigs.Exists(config => config.HeaterProfile == profile))
                measDuration += _heaterConfigs.Single(config => config.HeaterProfile == profile).HeaterDuration;

            return (int)Math.Ceiling(measDuration);
        }

        /// <summary>
        /// Sets a gas-sensor-heater profile. Converts the target temperature and duration to internally used format.
        /// </summary>
        /// <param name="profile">The selected profile to save to.</param>
        /// <param name="targetTemperature">The target temperature in °C. Ranging from 0-400.</param>
        /// <param name="duration">The duration in ms. Ranging from 0-4032.</param>
        /// <param name="ambientTemperature">The ambient temperature.</param>
        /// <returns></returns>
        public HeaterProfileConfiguration SaveHeaterProfileToDevice(HeaterProfile profile, ushort targetTemperature, ushort duration, double ambientTemperature)
        {
            // read ambient temperature for resistance calculation
            var heaterResistance = CalculateHeaterResistance(targetTemperature, (short)ambientTemperature);
            var heaterDuration = CalculateHeaterDuration(duration);

            Write8BitsToRegister((byte)((byte)Register.RES_HEAT0 + profile), heaterResistance);
            Write8BitsToRegister((byte)((byte)Register.GAS_WAIT0 + profile), heaterDuration);

            return new HeaterProfileConfiguration(profile, heaterResistance, heaterDuration);
        }

        /// <summary>
        /// Gets the specified gas-sensor-heater configuration.
        /// </summary>
        /// <param name="profile">The chosen heater profile.</param>
        /// <returns>The configuration of the chosen set-point or null if invalid set-point was chosen.</returns>
        public HeaterProfileConfiguration GetHeaterProfileFromDevice(HeaterProfile profile)
        {
            // need to be converted?!
            var heaterTemp = Read8BitsFromRegister((byte)((byte)Register.RES_HEAT0 + profile));
            var heaterDuration = Read8BitsFromRegister((byte)((byte)Register.GAS_WAIT0 + profile));

            return new HeaterProfileConfiguration(profile, heaterTemp, heaterDuration);
        }

        private Bme680ReadResult ReadResultRegisters()
        {
            var temp = ReadTemperature();
            var hum = ReadHumidity();
            var press = ReadPressure();
            var gasRes = ReadGasResistance();

            return new Bme680ReadResult
            {
                Temperature = temp,
                Humidity = hum,
                Pressure = press,
                GasResistance = gasRes
            };
        }

        /// <summary>
        /// Sets the power mode to the given mode.
        /// </summary>
        /// <param name="powerMode"></param>
        private void SetPowerMode(PowerMode powerMode)
        {
            var status = Read8BitsFromRegister((byte)Register.CTRL_MEAS);
            status = (byte)((status & (byte)~Mask.PWR_MODE) | (byte)powerMode);

            Write8BitsToRegister((byte)Register.CTRL_MEAS, status);
        }

        /// <summary>
        /// Set default configuration for basic measurements
        /// </summary>
        private void SetDefaultConfiguration()
        {
            // Read temperature before setting other sampling rates for faster measurement
            TemperatureSampling = Sampling.X1;
            PerformMeasurementAsync().Wait();
            var temp = ReadTemperature();

            // Set remaining sampling rates and filter
            HumiditySampling = Sampling.X1;
            PressureSampling = Sampling.X1;
            FilterCoefficient = FilterCoefficient.C0;

            // Set basic heater profile
            GasConversionIsEnabled = true;
            SaveHeaterProfileToDevice(HeaterProfile.Profile1, 320, 150, temp);
            CurrentHeaterProfile = HeaterProfile.Profile1;
        }

        /// <summary>
        /// Reads the temperature from the sensor.
        /// </summary>
        /// <returns>Temperature in degrees Celsius.</returns>
        private double ReadTemperature()
        {
            if (TemperatureSampling == Sampling.Skipped)
                return double.NaN;

            // Read 20 bit uncompensated temperature value from registers
            var t1 = Read16BitsFromRegister((byte)Register.TEMP, Endianness.BigEndian);
            var t2 = Read8BitsFromRegister((byte)Register.TEMP + 2 * sizeof(byte));

            // Combine the two values, t2 is only 4 bit
            var temp = (uint)(t1 << 4) + (uint)(t2 >> 4);

            return CalculateTemperature(temp);
        }

        /// <summary>
        ///  Reads the pressure from the sensor.
        /// </summary>
        /// <returns>Atmospheric pressure in Pa.</returns>
        private double ReadPressure()
        {
            if (_temperatureFine == int.MinValue)
                return double.NaN;

            if (PressureSampling == Sampling.Skipped)
                return double.NaN;

            // Read 20 bit uncompensated pressure value from registers
            var p1 = Read16BitsFromRegister((byte)Register.PRESS, Endianness.BigEndian);
            var p2 = Read8BitsFromRegister((byte)Register.PRESS + 2 * sizeof(byte));

            // Combine the two values, p2 is only 4 bit
            var press = (uint)(p1 << 4) + (uint)(p2 >> 4);

            return CalculatePressure(press);
        }

        /// <summary>
        /// Reads the humidity from the sensor.
        /// </summary>
        /// <returns>Humidity in percent.</returns>
        private double ReadHumidity()
        {
            if (_temperatureFine == int.MinValue)
                return int.MinValue;

            if (HumiditySampling == Sampling.Skipped)
                return double.NaN;

            // Read 16 bit uncompensated humidity value from registers
            var hum = Read16BitsFromRegister((byte)Register.HUM, Endianness.BigEndian);

            return CalculateHumidity(hum);
        }

        /// <summary>
        /// Reads the gas resistance from the sensor.
        /// </summary>
        /// <returns>Gas resistance in Ohm.</returns>
        private double ReadGasResistance()
        {
            if (!GasMeasurementIsValid)
            {
                return double.NaN;
            }

            // Read 10 bit gas resistance value from registers
            var g1 = Read8BitsFromRegister((byte)Register.GAS_RES);
            var g2 = Read8BitsFromRegister((byte)Register.GAS_RES + sizeof(byte));
            var gasRange = Read8BitsFromRegister((byte)Register.GAS_RANGE);

            var gasResistance = (ushort)((ushort)(g1 << 2) + (byte)(g2 >> 6));
            gasRange &= (byte)Mask.GAS_RANGE;

            return CalculateGasResistance(gasResistance, gasRange);
        }

        /// <summary>
        ///  Calculates the temperature in °C.
        /// </summary>
        /// <param name="adcTemperature">The temperature value read from the device.</param>
        /// <returns>Temperature</returns>
        private double CalculateTemperature(uint adcTemperature)
        {
            var var1 = (adcTemperature / 16384.0 - _calibrationData.ParT1 / 1024.0) * _calibrationData.ParT2;
            var var2 = adcTemperature / 131072.0 - _calibrationData.ParT1 / 8192.0;
            var2 = var2 * var2 * (_calibrationData.ParT3 * 16.0);

            _temperatureFine = (int)(var1 + var2);
            var temperature = (var1 + var2) / 5120.0;

            return temperature;
        }

        /// <summary>
        ///  Returns the pressure in Pa.
        /// </summary>
        /// <param name="adcPressure">The pressure value read from the device.</param>
        /// <returns>Pressure in Pa</returns>
        private double CalculatePressure(uint adcPressure)
        {
            var var1 = _temperatureFine / 2.0 - 64000.0;
            var var2 = var1 * var1 * (_calibrationData.ParP6 / 131072.0);
            var2 += var1 * _calibrationData.ParP5 * 2.0;
            var2 = var2 / 4.0 + _calibrationData.ParP4 * 65536.0;
            var1 = (_calibrationData.ParP3 * var1 * var1 / 16384.0 + _calibrationData.ParP2 * var1) / 524288.0;
            var1 = (1.0 + var1 / 32768.0) * _calibrationData.ParP1;
            var pressure = 1048576.0 - adcPressure;

            // ReSharper disable once CompareOfFloatsByEqualityOperator
            if (var1 == 0)
                return 0;

            pressure = (pressure - var2 / 4096.0) * 6250.0 / var1;
            var1 = _calibrationData.ParP9 * pressure * pressure / 2147483648.0;
            var2 = pressure * (_calibrationData.ParP8 / 32768.0);
            var var3 = (pressure / 256.0) * (pressure / 256.0) * (pressure / 256.0) * (_calibrationData.ParP10 / 131072.0);
            pressure += (var1 + var2 + var3 + _calibrationData.ParP7 * 128.0) / 16.0;

            return pressure;
        }

        private double CalculateHumidity(ushort adcHumidity)
        {
            var tempComp = _temperatureFine / 5120.0;
            var var1 = adcHumidity - (_calibrationData.ParH1 * 16.0 + _calibrationData.ParH3 / 2.0 * tempComp);
            var var2 = var1 * (_calibrationData.ParH2 / 262144.0 * (1.0 + _calibrationData.ParH4 / 16384.0 * tempComp + _calibrationData.ParH5 / 1048576.0 * (tempComp * tempComp)));
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

        private double CalculateGasResistance(ushort adcGasRes, byte gasRange)
        {
            var var1 = 1340.0 + 5.0 * _calibrationData.RangeSwErr;
            var var2 = var1 * (1.0 + K1Lookup[gasRange] / 100.0);
            var var3 = 1.0 + K2Lookup[gasRange] / 100.0;
            var gasResistance = 1.0 / (var3 * 0.000000125 * (1 << gasRange) * ((adcGasRes - 512.0) / var2 + 1.0));

            return gasResistance;
        }

        private byte CalculateHeaterResistance(ushort setTemp, short ambientTemp)
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


        // The duration is interpreted as follows:
        // Byte [7:6]: multiplication factor of 1 ,4, 16 or 64
        // Byte [5:0]: 64 timer values, 1ms step size
        // Values are rounded down
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

            return mode switch
            {
                Endianness.LittleEndian => BinaryPrimitives.ReadUInt16LittleEndian(bytes),
                Endianness.BigEndian => BinaryPrimitives.ReadUInt16BigEndian(bytes),
                _ => throw new ArgumentOutOfRangeException(nameof(mode), mode, null)
            };
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

        /// <summary>
        /// Disposes the Bme680 resources.
        /// </summary>
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
