# BME680

C# driver for the Bosch BME680 temperature/humidity/pressure/air quality sensor. This driver is ported from the [Bosch C driver](https://github.com/BoschSensortec/BME680_driver).

The datasheet can be found [here](https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME680-DS001.pdf).

## Dependencies

This library depends on __System.Device.Gpio__ and __Iot.Units__ namespaces for communication over I²C and some helpful unit wrappers. To get these libraries you will need to add https://dotnetfeed.blob.core.windows.net/dotnet-iot/index.json to your nuget sources since they are currently in prerelease status.

## Example on how to use this library

The process of using this library is straight forward, just reference it as dependency in your project or copy all the files to your project. You can either use the default settings of the sensor which will perform a measurement of temperature, humidity, pressure and gas (volatile organic compounds) or change those settings to suit your use case.

To use the default settings you could do something like this:

```C#
class Program
    {
        static async Task Main(string[] args)
        {
            // create the device
            var settings = new I2cConnectionSettings(1, Bme680.DefaultI2cAddress);
            var device = new UnixI2cDevice(settings);
            var bme680 = new Bme680(device);

            // Make the device ready for use
            bme680.InitDevice();

            Console.WriteLine("Performing measurements with the default configuration:\n");
            while (true)
            {
                // perform measurement
                await bme680.PerformMeasurement();

                // read results from registers
                var temp = bme680.Temperature;
                var press = bme680.Pressure;
                var hum = bme680.Humidity;
                var gasRes = bme680.GasResistance;

                Console.WriteLine($"Temperature: {temp.Celsius}°C\nPressure: {press}\nHumidity: {hum}\nGas Resistance: {gasRes}\n");
                Task.Delay(1000).Wait();
            }
        }
    }

```

To use your custom settings you could do something like this:

```C#
class Program
    {
        static async Task Main(string[] args)
        {
            // create the device
            var settings = new I2cConnectionSettings(1, Bme680.DefaultI2cAddress);
            var device = new UnixI2cDevice(settings);
            var bme680 = new Bme680(device);

            // Make the device ready for use
            bme680.InitDevice();
        
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
                // Get the time the measurement will take
                var duration = bme680.GetProfileDuration(bme680.CurrentHeaterProfile);

                // perform the measurement by setting the power mode
                bme680.SetPowerMode(PowerMode.Forced);

               // wait for the measurement to finish
                Task.Delay(duration).Wait();

               // it's also possible to wait by polling the status of the measurement
               //while (!bme680.NewDataIsAvailable){}

                // read results from registers
                var temp = bme680.Temperature;
                var press = bme680.Pressure;
                var hum = bme680.Humidity;
                var gasRes = bme680.GasResistance;
                
                // it can make sense to update the heater profile continually since the ambient temperature
                // is taken into account when the heater profile is set
                bme680.SaveHeaterProfileToDevice(HeaterProfile.Profile3, 330, 120, temp);

                Console.WriteLine($"Temperature: {temp.Celsius}°C\nPressure: {press}\nHumidity: {hum}\nGas Resistance: {gasRes}\n");
                Task.Delay(1000).Wait();
            }
        }
    }
```
