using System.Collections.Generic;
using PathFinder.Interfaces;
using PathFinder.Models;

namespace PathFinder.Strategies
{
    public class RaycastingPathOptimizer : IPathOptimizer
    {
        private readonly ILineOfSightChecker _lineOfSightChecker;

        public RaycastingPathOptimizer(ILineOfSightChecker lineOfSightChecker)
        {
            _lineOfSightChecker = lineOfSightChecker;
        }

        public List<Coordinate> Optimize(List<Coordinate> path, int[,] grid)
        {
            if (path == null || path.Count < 3)
                return path;

            var waypoints = new List<Coordinate>();

            waypoints.Add(path[0]);
            int lastWaypointIndex = 0;

            for (int i = 1; i < path.Count; i++)
            {
                // Aggiungiamo 200 come default per MaxAllowCost o recuperiamo dalla configurazione
                bool isVisible = _lineOfSightChecker.HasLineOfSight(grid, path[lastWaypointIndex], path[i], 200);

                if (!isVisible)
                {
                    waypoints.Add(path[i - 1]);
                    lastWaypointIndex = i - 1;
                }
            }

            waypoints.Add(path[path.Count - 1]);

            return waypoints;
        }
    }
}