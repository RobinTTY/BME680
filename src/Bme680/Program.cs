using System;
using System.Device.I2c;
using System.Device.I2c.Drivers;
using System.Threading.Tasks;
using Bme680;

namespace Bme680
{
    class Program
    {
        // TODO: check what happens if t/p/h/g is read before initializing
        static async Task Main(string[] args)
        {
            var settings = new I2cConnectionSettings(1, 0x76);
            var device = new UnixI2cDevice(settings);
            var bme680 = new Bme680(device);

            var debug = true;
            while (debug)
            {
                Task.Delay(1000).Wait();
            }

            bme680.InitDevice();

            // 3.2.1 Quick start example (page 15)
            while (true)
            {
                // set device settings
                
                bme680.HumiditySampling = Sampling.X1;
                bme680.TemperatureSampling = Sampling.X2;
                bme680.PressureSampling = Sampling.X16;
                bme680.FilterCoefficient = FilterCoefficient.C3;
                
                // set settings for gas conversion
                bme680.GasConversionIsEnabled = true;
                bme680.SaveHeaterProfileToDevice(HeaterProfile.Profile1, 300, 100, 24.0);
                bme680.CurrentHeaterProfile = HeaterProfile.Profile1;
                
                // perform measurements
                await bme680.PerformMeasurement();

                // read results from registers
                var temp = bme680.ReadTemperature();
                var press = bme680.ReadPressure();
                var hum = bme680.ReadHumidity();
                var gasRes = bme680.ReadGasResistance();

                Console.WriteLine($"Temperature: {temp}\nPressure: {press}\nHumidity: {hum}\nGas Resistance: {gasRes}\n");
                Task.Delay(1000).Wait();
            }
        }
    }
}