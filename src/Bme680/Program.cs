using System;
using System.Device.I2c;
using System.Device.I2c.Drivers;
using System.Threading.Tasks;
using Bme680;

namespace Bme680
{
    class Program
    {
        static void Main(string[] args)
        {
            var settings = new I2cConnectionSettings(1, 0x76);
            var device = new UnixI2cDevice(settings);
            var bme680 = new Bme680(device);

            //var debug = true;
            //while (debug)
            //{
            //    Task.Delay(1000).Wait();
            //}

            // 3.2.1 Quick start example (page 15)
            for (var i = 0; i < 5; i++)
            {
                // set device settings
                bme680.InitDevice();
                bme680.HumiditySampling = Sampling.X1;
                bme680.TemperatureSampling = Sampling.X2;
                bme680.PressureSampling = Sampling.X16;
                bme680.FilterCoefficient = FilterCoefficient.C3;

                bme680.GasConversionIsEnabled = true;
                bme680.ConfigureHeaterProfile(HeaterProfile.Profile1, 24.0, 300, 100);
                bme680.CurrentHeaterProfile = HeaterProfile.Profile1;
                bme680.HeaterIsEnabled = true;

                // perform measurements
                bme680.SetPowerMode(PowerMode.Forced);

                // TODO: wait timing?!?!?!?!
                var temp = bme680.ReadTemperature();
                var press = bme680.ReadPressure();
                var hum = bme680.ReadHumidity();
                var gasRes = bme680.ReadGasResistance();

                Console.WriteLine($"Temperature: {temp}\nPressure: {press}\nHumidity: {hum}\nGas Resistance: {gasRes}\n");
            }
        }
    }
}