namespace Bme680
{
    public enum WaitTimeMultiplier : byte
    {
        /// <summary>
        /// Multiplication factor of 1
        /// </summary>
        M1 = 0b00,
        /// <summary>
        /// Multiplication factor of 4
        /// </summary>
        M4 = 0b01,
        /// <summary>
        /// Multiplication factor of 16
        /// </summary>
        M16 = 0b10,
        /// <summary>
        /// Multiplication factor of 64
        /// </summary>
        M64 = 0b11
    }
}
