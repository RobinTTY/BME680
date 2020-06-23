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
            var settings = new I2cConnectionSettings(1, Bme680.SecondaryI2cAddress);
            var i2CDevice = I2cDevice.Create(settings);
            using var bme680 = new Bme680(i2CDevice);

            while (true)
            {
                var measurement = await bme680.PerformMeasurementAsync();

                Console.WriteLine($"Temperature: {measurement.Temperature}");
                Console.WriteLine($"Humidity: {measurement.Humidity}");
                Console.WriteLine($"Pressure: {measurement.Pressure}");
                Console.WriteLine($"Gas Resistance: {measurement.GasResistance}");
                Console.WriteLine();

                await Task.Delay(1000);
            }
        }
    }
}
