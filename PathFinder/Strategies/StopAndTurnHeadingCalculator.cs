using PathFinder.Interfaces;
using PathFinder.Models;

namespace PathFinder.Strategies
{
    public class StopAndTurnHeadingCalculator : IHeadingCalculator
    {
        private readonly ISpeedEvaluator _speedEvaluator;
        private readonly double _initialRobotHeading = 0.0;

        // Iniettiamo l'evaluator anche qui
        public StopAndTurnHeadingCalculator(ISpeedEvaluator speedEvaluator)
        {
            _speedEvaluator = speedEvaluator;
        }

        // Aggiungiamo la costmap alla firma per poter leggere i costi
        public List<Pose> CalculateHeadings(List<Coordinate> waypoints, int[,] costmap, double? finalHeadingRad = null)
        {
            var poses = new List<Pose>();
            if (waypoints == null || waypoints.Count == 0) return poses;

            if (waypoints.Count == 1)
            {
                double speed = _speedEvaluator.GetSpeedForCost(costmap[waypoints[0].X, waypoints[0].Y]);
                double initialRotation = finalHeadingRad ?? 0;
                poses.Add(new Pose(waypoints[0].X, waypoints[0].Y, initialRotation, speed));
                return poses;
            }

            double currentHeading = _initialRobotHeading;

            for (int i = 0; i < waypoints.Count - 1; i++)
            {
                var current = waypoints[i];
                var next = waypoints[i + 1];

                double dy = next.Y - current.Y;
                double dx = next.X - current.X;

                double targetHeading = Math.Atan2(dy, dx);
                double rawDeltaTheta = targetHeading - currentHeading;
                double normalizedDeltaTheta = Math.Atan2(Math.Sin(rawDeltaTheta), Math.Cos(rawDeltaTheta));

                // CALCOLO VELOCITÀ: Leggiamo il costo del waypoint attuale e assegnamo la marcia!
                double nodeSpeed = _speedEvaluator.GetSpeedForCost(costmap[current.X, current.Y]);

                poses.Add(new Pose(current.X, current.Y, normalizedDeltaTheta, nodeSpeed));
                currentHeading = targetHeading;
            }

            var lastCoord = waypoints[^1];
            double finalRotation = finalHeadingRad.HasValue
                ? Math.Atan2(Math.Sin(finalHeadingRad.Value - currentHeading), Math.Cos(finalHeadingRad.Value - currentHeading))
                : 0.0;

            // La velocità di arrivo è sempre 0, il robot deve fermarsi.
            poses.Add(new Pose(lastCoord.X, lastCoord.Y, finalRotation, 0.0));

            return poses;
        }
    }
}