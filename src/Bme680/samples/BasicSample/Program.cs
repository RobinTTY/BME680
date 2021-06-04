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
            var settings = new I2cConnectionSettings(1, Bme680.DefaultI2cAddress);
            var i2CDevice = I2cDevice.Create(settings);
            using var bme680 = new Bme680(i2CDevice);

            while (true)
            {
                var measurement = await bme680.PerformMeasurementAsync();

                Console.WriteLine($"Temperature: {measurement.Temperature:0.##}°C");
                Console.WriteLine($"Humidity: {measurement.Humidity:0.##}%");
                Console.WriteLine($"Pressure: {measurement.Pressure:0.##} Pa");
                Console.WriteLine($"Gas Resistance: {measurement.GasResistance:0.##} Ohm");
                Console.WriteLine();

                await Task.Delay(1000);
            }
        }
    }
}
