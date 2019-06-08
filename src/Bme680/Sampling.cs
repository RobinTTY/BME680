namespace Bme680Driver
{
    public enum Sampling : byte
    {
        /// <summary>
        /// Skipped (output set to 0x8000)
        /// </summary>
        Skipped = 0b000,

        /// <summary>
        /// oversampling x1
        /// </summary>
        X1 = 0b001,
        
        /// <summary>
        /// oversampling x2
        /// </summary>
        X2 = 0b010,
        
        /// <summary>
        /// oversampling x4
        /// </summary>
        X4 = 0b011,
        
        /// <summary>
        /// oversampling x8
        /// </summary>
        X8 = 0b100,
        
        /// <summary>
        /// oversampling x16
        /// </summary>
        X16 = 0b101
    }
}
