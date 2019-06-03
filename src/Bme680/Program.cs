using System;
using System.Device.I2c;
using System.Device.I2c.Drivers;
using System.Threading.Tasks;

namespace Bme680
{
    class Program
    {
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

            // 3.2.1 Quick start example (page 15)
            for (var i = 0; i < 5; i++)
            {
                bme680.InitDevice();
                bme680.HumiditySampling = Sampling.X1;
                bme680.TemperatureSampling = Sampling.X2;
                bme680.PressureSampling = Sampling.X16;

                await bme680.SetGasConfig(HeaterProfile.Profile1, 300, 100);
                bme680.SelectHeaterProfile(HeaterProfile.Profile1);
                bme680.GasConversionIsEnabled = true;
                bme680.HeaterIsEnabled = true;

                // TODO: wait timing?!?!?!?!
                var temp = await bme680.ReadTemperatureAsync();
                var press = await bme680.ReadPressureAsync();
                var hum = await bme680.ReadHumidityAsync();
                var gasRes = await bme680.ReadGasResistanceAsync();

                Console.WriteLine($"Temperature: {temp}\nPressure: {press}\nHumidity: {hum}\nGas Resistance: {gasRes}\n");
            }
        }
    }
}
