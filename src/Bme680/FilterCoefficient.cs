namespace Bme680Driver
{
    /// <summary>
    /// IIR filter coefficient.
    /// </summary>
    public enum FilterCoefficient : byte
    {
        /// <summary>
        /// Filter coefficient of 0
        /// </summary>
        C0 = 0b000,
        /// <summary>
        /// Filter coefficient of 1
        /// </summary>
        C1 = 0b001,
        /// <summary>
        /// Filter coefficient of 3
        /// </summary>
        C3 = 0b010,
        /// <summary>
        /// Filter coefficient of 7
        /// </summary>
        C7 = 0b011,
        /// <summary>
        /// Filter coefficient of 15
        /// </summary>
        C15 = 0b100,
        /// <summary>
        /// Filter coefficient of 31
        /// </summary>
        C31 = 0b101,
        /// <summary>
        /// Filter coefficient of 63
        /// </summary>
        C63 = 0b110,
        /// <summary>
        /// Filter coefficient of 127
        /// </summary>
        C127 = 0b111
    }
}
