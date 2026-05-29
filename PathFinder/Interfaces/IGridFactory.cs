using System.IO;
using System.Threading.Tasks;
using PathFinder.Models;

namespace PathFinder.Interfaces
{
    public interface IGridFactory
    {
        Task<int[,]> ParseImageAsync(Stream imageStream, int cellSize);
        int[,] InflateGrid(int[,] rawGrid, int robotRadiusCells);
    }
}