using System;
using System.Collections.Generic;
using System.Text;

namespace Bme680
{
    public class GasConfiguration
    {
        internal byte NbConv;
        public ushort HeaterTemp { get; set; }
        public ushort HeaterDur { get; set; }

        public GasConfiguration(ushort heaterTemp, ushort heaterDur)
        {
            HeaterTemp = heaterTemp;
            HeaterDur = heaterDur;
        }
    }
}
