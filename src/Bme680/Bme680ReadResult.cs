namespace Bme680Driver
{
    /// <summary>
    /// The measurements a Bme680 sensor will return.
    /// </summary>
    public struct Bme680ReadResult
    {
        /// <summary>
        /// Temperature in degrees Celsius.
        /// </summary>
        public double Temperature { get; set; }
        /// <summary>
        /// Relative humidity.
        /// </summary>
        public double Humidity { get; set; }
        /// <summary>
        /// Pressure in Pascal.
        /// </summary>
        public double Pressure { get; set; }
        /// <summary>
        /// Gets the last measured gas resistance in Ohm from the corresponding register.
        /// </summary>
        public double GasResistance { get; set; }
    }
}
