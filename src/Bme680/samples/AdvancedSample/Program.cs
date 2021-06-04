using System;
using System.Device.I2c;
using System.Threading.Tasks;
using Bme680Driver;

namespace Bme680Sample
{
    class Program
    {
        static async Task Main(string[] args)
        {
            // The BME680 uses either 0x76 or 0x77 as I2C address, the address has to be specified here
            var settings = new I2cConnectionSettings(1, Bme680.SecondaryI2cAddress);
            var i2CDevice = I2cDevice.Create(settings);
            using var bme680 = new Bme680(i2CDevice);

            // set custom device settings (A higher sampling also increases the time a measurement will take)
            // A sampling rate of X4 will take roughly 4 times longer than a sampling rate of X1
            // You can find out how long a measurement will take by using the method GetProfileDuration()
            bme680.HumiditySampling = Sampling.X4;
            bme680.TemperatureSampling = Sampling.X1;
            bme680.PressureSampling = Sampling.Skipped;
            bme680.FilterCoefficient = FilterCoefficient.C31;

            // set custom settings for gas conversion
            bme680.GasConversionIsEnabled = true;
            bme680.HeaterIsEnabled = true;
            
            // The BME680 sensor can save up to 10 heater profiles for use                
            // this profile will set the target temperature of the heating plate to 330°C
            // with a heating duration of 120ms and an ambient temperature of 24.0°C
            bme680.SaveHeaterProfileToDevice(HeaterProfile.Profile3, 330, 120, 24.0);
            bme680.CurrentHeaterProfile = HeaterProfile.Profile3;

            Console.WriteLine("Performing measurements with custom configuration:\n");
            while (true)
            {
                // perform the measurement
                var measurement = await bme680.PerformMeasurementAsync();

                // print results
                Console.WriteLine($"Temperature: {measurement.Temperature:0.##}°C");
                Console.WriteLine($"Humidity: {measurement.Humidity:0.##}%");
                Console.WriteLine($"Pressure: {measurement.Pressure:0.##} Pa");
                Console.WriteLine($"Gas Resistance: {measurement.GasResistance:0.##} Ohm");
                Console.WriteLine();

                // it can make sense to update the heater profile continually since the ambient temperature
                // is taken into account when the heater profile is set
                bme680.SaveHeaterProfileToDevice(HeaterProfile.Profile3, 330, 120, measurement.Temperature);

                Task.Delay(1000).Wait();
            }
        }
    }
}
