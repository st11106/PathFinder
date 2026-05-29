using PathFinder.Interfaces;
using PathFinder.Models;

namespace PathFinder.Services
{
    public class PathfindingService
    {
        private readonly IGridFactory _gridFactory;
        private readonly IPathfindingEngine _engine;
        private readonly IPathOptimizer _optimizer;
        private readonly IHeadingCalculator _headingCalculator;

        public PathfindingService(
            IGridFactory gridFactory,
            IPathfindingEngine engine,
            IPathOptimizer optimizer,
            IHeadingCalculator headingCalculator)
        {
            _gridFactory = gridFactory;
            _engine = engine;
            _optimizer = optimizer;
            _headingCalculator = headingCalculator;
        }

        public async Task<List<Pose>> CalculatePathAsync(
            Stream imageStream,
            List<Coordinate> targetCells,
            int cellSize,
            int robotRadius,
            double? finalHeadingRad = null)
        {
            var rawGrid = await _gridFactory.ParseImageAsync(imageStream, cellSize);
            var inflatedGrid = _gridFactory.InflateGrid(rawGrid, robotRadius);

            var fullOptimizedPath = new List<Coordinate>();

            for (int i = 0; i < targetCells.Count - 1; i++)
            {
                var startNode = targetCells[i];
                var endNode = targetCells[i + 1];

                var segmentPath = _engine.FindPath(inflatedGrid, startNode, endNode);

                if (segmentPath == null || segmentPath.Count == 0)
                    return new List<Pose>(); // Errore: Tappa non raggiungibile

                var optimizedSegment = _optimizer.Optimize(segmentPath, inflatedGrid);

                // 3. Uniamo il segmento ottimizzato al percorso globale
                if (i > 0)
                {
                    optimizedSegment.RemoveAt(0);
                }

                fullOptimizedPath.AddRange(optimizedSegment);
            }
            var finalPoses = _headingCalculator.CalculateHeadings(fullOptimizedPath, inflatedGrid, finalHeadingRad);

            return finalPoses;
        }
    }
}