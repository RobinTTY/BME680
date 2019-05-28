using System.Device.I2c;
using System.Device.I2c.Drivers;
using System.Threading.Tasks;

namespace Bme680
{
    class Program
    {
        static void Main(string[] args)
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
        }
    }
}
