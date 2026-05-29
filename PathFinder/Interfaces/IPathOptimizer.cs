using System.Collections.Generic;
using PathFinder.Models;

namespace PathFinder.Interfaces
{
    public interface IPathOptimizer
    {
        List<Coordinate> Optimize(List<Coordinate> path, int[,] grid);
    }
}