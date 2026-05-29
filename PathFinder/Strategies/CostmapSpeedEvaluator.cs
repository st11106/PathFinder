using PathFinder.Interfaces;
using PathFinder.Models;

namespace PathFinder.Strategies
{
    public class CostmapSpeedEvaluator : ISpeedEvaluator
    {
        private readonly List<SpeedZone> _zones;

        public CostmapSpeedEvaluator()
        {
            // Ecco le tue 5 VELOCITÀ. 
            // In un'app Enterprise, questi dati verrebbero letti da appsettings.json!
            _zones = new List<SpeedZone>
            {
                new SpeedZone { MaxCost = 10,  Speed = 1.0 }, // Zona larghissima
                new SpeedZone { MaxCost = 35,  Speed = 0.8 }, // Zona comoda
                new SpeedZone { MaxCost = 50, Speed = 0.6 }, // Zona media
                new SpeedZone { MaxCost = 75, Speed = 0.4 }, // Zona stretta
                new SpeedZone { MaxCost = 100, Speed = 0.2 }  // Zona critica / Rasente al muro
            };

            // Ordiniamo per sicurezza dal costo minore al maggiore
            _zones = _zones.OrderBy(z => z.MaxCost).ToList();
        }

        public double GetSpeedForCost(int cost)
        {
            // Scorre le zone e restituisce la velocità della prima zona in cui il costo rientra
            foreach (var zone in _zones)
            {
                if (cost <= zone.MaxCost)
                    return zone.Speed;
            }

            return 0.0; // Se supera il costo massimo (es. 255 = Muro), il robot si ferma.
        }

        public bool IsCostAllowed(int cost, int maxAllowedCost)
        {
            return cost <= maxAllowedCost;
        }
    }
}