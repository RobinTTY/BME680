namespace Bme680Driver
{
    /// <summary>
    /// BMP680s power modes
    /// </summary>
    public enum PowerMode : byte
    {
        /// <summary>
        /// Power saving mode, no measurements are performed
        /// </summary>
        Sleep = 0b00,
        /// <summary>
        /// Device goes to sleep mode after one measurement
        /// </summary>
        Forced = 0b01
    }
}
