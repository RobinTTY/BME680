namespace Bme680
{
    public class HeaterProfileConfiguration
    {
        public ushort HeaterTemp { get; set; }
        public ushort HeaterDur { get; set; }

        public HeaterProfileConfiguration(ushort heaterTemp, ushort heaterDur)
        {
            HeaterTemp = heaterTemp;
            HeaterDur = heaterDur;
        }
    }
}
