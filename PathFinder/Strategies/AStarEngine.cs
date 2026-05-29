using PathFinder.Interfaces;
using PathFinder.Models;

namespace PathFinder.Strategies
{
    public class AStarEngine : IPathfindingEngine
    {
        public List<Coordinate> FindPath(int[,] grid, Coordinate start, Coordinate end)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);

            // Consideriamo ostacolo se il costo supera 200 (valore di default per IsCostAllowed) 
            // In un'app reale, questo dovrebbe essere letto dalla velocità / strategia o iniettato
            if (!IsValid(start, width, height) || !IsValid(end, width, height) ||
                grid[start.X, start.Y] > 200 || grid[end.X, end.Y] > 200)
            {
                return new List<Coordinate>();
            }

            var openQueue = new PriorityQueue<PathNode, int>();
            var allNodes = new Dictionary<Coordinate, PathNode>();
            var closedSet = new HashSet<Coordinate>();

            var startNode = new PathNode(start) { GCost = 0, HCost = GetOctileHeuristic(start, end) };
            allNodes[start] = startNode;
            openQueue.Enqueue(startNode, startNode.FCost);

            while (openQueue.Count > 0)
            {
                var current = openQueue.Dequeue();

                if (!closedSet.Add(current.Position)) continue;

                if (current.Position == end)
                {
                    return RetracePath(current);
                }

                foreach (var neighborPos in GetNeighbors(current.Position, width, height))
                {
                    int cellCost = grid[neighborPos.X, neighborPos.Y];
                    if (closedSet.Contains(neighborPos) || cellCost > 200)
                        continue;

                    // Il costo di movimento base (10 = dritto, 14 = diag)
                    int movementCost = IsDiagonal(current.Position, neighborPos) ? 14 : 10;

                    // Aggiungiamo un forte peso basato sulla costmap
                    // Moltiplichiamo il valore della costmap (0-200) per forzare A* a evitarlo
                    // Puoi aumentare questo moltiplicatore per renderlo ancora più "timoroso" dei muri
                    int penalty = Convert.ToInt32(cellCost * 4);

                    int newGCost = current.GCost + movementCost + penalty;

                    if (!allNodes.TryGetValue(neighborPos, out var neighborNode))
                    {
                        neighborNode = new PathNode(neighborPos);
                        allNodes[neighborPos] = neighborNode;
                    }

                    if (newGCost < neighborNode.GCost || !neighborNode.InOpenList)
                    {
                        neighborNode.GCost = newGCost;
                        neighborNode.HCost = GetOctileHeuristic(neighborPos, end);
                        neighborNode.Parent = current;

                        openQueue.Enqueue(neighborNode, neighborNode.FCost);
                        neighborNode.InOpenList = true;
                    }
                }
            }

            return new List<Coordinate>();
        }

        private int GetOctileHeuristic(Coordinate a, Coordinate b)
        {
            int dx = Math.Abs(a.X - b.X);
            int dy = Math.Abs(a.Y - b.Y);
            int h = 10 * (dx + dy) + (14 - 2 * 10) * Math.Min(dx, dy);
            return h + (h / 1000);
        }

        private IEnumerable<Coordinate> GetNeighbors(Coordinate current, int width, int height)
        {
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    if (x == 0 && y == 0) continue;

                    int newX = current.X + x;
                    int newY = current.Y + y;

                    if (IsValid(new Coordinate(newX, newY), width, height))
                    {
                        yield return new Coordinate(newX, newY);
                    }
                }
            }
        }

        private bool IsValid(Coordinate c, int width, int height)
        {
            return c.X >= 0 && c.X < width && c.Y >= 0 && c.Y < height;
        }

        private bool IsDiagonal(Coordinate a, Coordinate b)
        {
            return Math.Abs(a.X - b.X) == 1 && Math.Abs(a.Y - b.Y) == 1;
        }

        private List<Coordinate> RetracePath(PathNode endNode)
        {
            var path = new List<Coordinate>();
            var current = endNode;

            while (current != null)
            {
                path.Add(current.Position);
                current = current.Parent;
            }

            path.Reverse();
            return path;
        }
    }
}