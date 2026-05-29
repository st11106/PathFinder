using PathFinder.Interfaces;
using SkiaSharp;

namespace PathFinder.Strategies
{
    public class GridFactory : IGridFactory
    {
        public async Task<int[,]> ParseImageAsync(Stream imageStream, int cellSize)
        {
            if (cellSize <= 0) cellSize = 1;

            using var bitmap = SKBitmap.Decode(imageStream);
            var pixels = bitmap.Pixels;

            int origWidth = bitmap.Width;
            int origHeight = bitmap.Height;

            int newWidth = (int)Math.Ceiling((double)origWidth / cellSize);
            int newHeight = (int)Math.Ceiling((double)origHeight / cellSize);

            var rawGrid = new int[newWidth, newHeight];

            for (int y = 0; y < newHeight; y++)
                for (int x = 0; x < newWidth; x++)
                    rawGrid[x, y] = 0; // 0 = Walkable (Free)

            for (int y = 0; y < origHeight; y++)
            {
                for (int x = 0; x < origWidth; x++)
                {
                    SKColor pixel = pixels[y * origWidth + x];
                    int gridX = x / cellSize;
                    int gridY = y / cellSize;

                    if (rawGrid[gridX, gridY] == 255) continue; // 255 = Ostacolo

                    if (pixel.Red < 128 && pixel.Green < 128 && pixel.Blue < 128)
                    {
                        rawGrid[gridX, gridY] = 255;
                    }
                }
            }

            return rawGrid;
        }

        public int[,] InflateGrid(int[,] rawGrid, int robotRadiusCells)
        {
            if (robotRadiusCells <= 0) return rawGrid;

            int width = rawGrid.GetLength(0);
            int height = rawGrid.GetLength(1);
            var inflatedGrid = new int[width, height];

            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    inflatedGrid[x, y] = 0; // Walkable

            int radiusSquared = robotRadiusCells * robotRadiusCells;

            // Definiamo un raggio di "inflation" per il decadimento della velocità,
            // ad esempio pari a 3 volte il raggio del robot.
            double inflationRadiusCells = robotRadiusCells * 3.0;
            int inflationRadiusSquared = (int)(inflationRadiusCells * inflationRadiusCells);

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    if (rawGrid[x, y] > 200) // Consideriamo ostacolo originale
                    {
                        inflatedGrid[x, y] = 255;

                        int maxRadiusCheck = (int)Math.Ceiling(inflationRadiusCells);

                        for (int dy = -maxRadiusCheck; dy <= maxRadiusCheck; dy++)
                        {
                            for (int dx = -maxRadiusCheck; dx <= maxRadiusCheck; dx++)
                            {
                                int distSquared = dx * dx + dy * dy;

                                if (distSquared <= inflationRadiusSquared)
                                {
                                    int nx = x + dx;
                                    int ny = y + dy;

                                    if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                                    {
                                        if (distSquared <= radiusSquared)
                                        {
                                            // 1. ZONA LETALE: Dentro il raggio rigido del robot
                                            inflatedGrid[nx, ny] = 255;
                                        }
                                        else if (inflatedGrid[nx, ny] != 255)
                                        {
                                            // 2. ZONA SFUMATA: Calcoliamo un decadimento basato sulla distanza
                                            double dist = Math.Sqrt(distSquared);

                                            // Costo decresce da 200 (margine raggio robot) verso 0 (margine raggio inflation)
                                            // y = 200 * (1 - (dist - R_robot) / (R_inflation - R_robot))
                                            double costFactor = 1.0 - ((dist - robotRadiusCells) / (inflationRadiusCells - robotRadiusCells));

                                            // Applichiamo una decadenza (clampato tra 1 e 200)
                                            int newCost = (int)Math.Max(1, Math.Min(200, costFactor * 200));

                                            // Applichiamo il costo maggiore:
                                            // Se il nodo era già sfiorato da un altro ostacolo che imponeva
                                            // un costo maggiore, manteniamo quello maggiore.
                                            if (newCost > inflatedGrid[nx, ny])
                                            {
                                                inflatedGrid[nx, ny] = newCost;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return inflatedGrid;
        }
    }
}