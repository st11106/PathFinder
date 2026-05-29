using PathFinder.Models;

namespace PathFinder.Interfaces
{
    public interface ILineOfSightChecker
    {
        bool HasLineOfSight(int[,] grid, Coordinate start, Coordinate end, int MaxAllowCost);
    }
}