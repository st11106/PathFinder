using PathFinder.Interfaces;
using PathFinder.Models;

namespace PathFinder.Strategies
{
    public class BresenhamLineOfSightChecker : ILineOfSightChecker
    {
        private readonly ISpeedEvaluator _speedEvaluator;

        // Iniezione della dipendenza
        public BresenhamLineOfSightChecker(ISpeedEvaluator speedEvaluator)
        {
            _speedEvaluator = speedEvaluator;
        }

        public bool HasLineOfSight(int[,] costmap, Coordinate start, Coordinate end, int maxAllowedCost = 200)
        {
            int x0 = start.X, y0 = start.Y;
            int x1 = end.X, y1 = end.Y;

            int dx = Math.Abs(x1 - x0);
            int dy = Math.Abs(y1 - y0);
            int sx = x0 < x1 ? 1 : -1;
            int sy = y0 < y1 ? 1 : -1;
            int err = dx - dy;

            // Chiediamo all'evaluator la velocità nel punto di partenza
            double startingSpeed = _speedEvaluator.GetSpeedForCost(costmap[x0, y0]);

            while (true)
            {
                int currentCost = costmap[x0, y0];

                // 1. Il costo supera la soglia di sicurezza? Spezza la linea.
                if (!_speedEvaluator.IsCostAllowed(currentCost, maxAllowedCost))
                    return false;

                // 2. La velocità ottimale è cambiata rispetto alla partenza? Spezza la linea!
                double currentSpeed = _speedEvaluator.GetSpeedForCost(currentCost);
                if (Math.Abs(currentSpeed - startingSpeed) > 0.01) // Confronto double sicuro
                    return false;

                if (x0 == x1 && y0 == y1) break;

                int e2 = 2 * err;
                if (e2 > -dy) { err -= dy; x0 += sx; }
                if (e2 < dx) { err += dx; y0 += sy; }
            }

            return true;
        }
    }
}