using System.Collections.Generic;
using PathFinder.Models;

namespace PathFinder.Interfaces
{
    public interface IPathfindingEngine
    {
        List<Coordinate> FindPath(int[,] grid, Coordinate start, Coordinate end);
    }
}