using PathFinder.Models;

namespace PathFinder.Interfaces
{
    public interface IHeadingCalculator
    {
        /// <summary>
        /// Presa una lista di waypoints (Coordinate) e restituisce una lista di Pose, dove ogni Pose include la posizione (X, Y) e l'orientamento (heading) calcolato in base alla direzione tra i waypoints.
        /// </summary>
        /// <param name="waypoints"></param>
        /// <returns></returns>
        List<Pose> CalculateHeadings(List<Coordinate> waypoints, int[,] costmap, double? finalHeadingRad = null);

    }
}
