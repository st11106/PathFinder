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

        /// <summary>
        /// Rimuove il rumore tipico delle mappe SLAM (pixel-ostacolo isolati).
        /// Una cella resta ostacolo solo se ha almeno <paramref name="minNeighbors"/>
        /// vicini (8-connessi) anch'essi ostacolo. Senza questo passaggio i puntini
        /// isolati, una volta dilatati dall'inflation, sbarrano tutti i corridoi e
        /// l'A* non trova alcun percorso.
        /// </summary>
        public int[,] DenoiseGrid(int[,] grid, int minNeighbors)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);
            var cleaned = new int[width, height];

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    if (grid[x, y] <= 200) { cleaned[x, y] = 0; continue; }

                    int neighbors = 0;
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            if (dx == 0 && dy == 0) continue;
                            int nx = x + dx, ny = y + dy;
                            if (nx >= 0 && nx < width && ny >= 0 && ny < height && grid[nx, ny] > 200)
                                neighbors++;
                        }
                    }

                    cleaned[x, y] = neighbors >= minNeighbors ? 255 : 0;
                }
            }

            return cleaned;
        }

        /// <summary>
        /// Dilata gli ostacoli (inflation) in base al raggio del robot.
        /// Prima applica un denoising del rumore SLAM, poi usa una Distance Transform
        /// (Chamfer 3-4) a due passate, O(N) anziché O(N * R^2), assegnando:
        ///   - 255 (zona letale / non percorribile) se dist &lt;= robotRadiusCells
        ///   - un costo decrescente 1..200 nella fascia di inflation
        ///   - 0 (libero) oltre la fascia di inflation
        /// </summary>
        public int[,] InflateGrid(int[,] rawGrid, int robotRadiusCells)
        {
            // Pulizia del rumore SLAM prima della dilatazione. minNeighbors=2 rimuove
            // i puntini isolati e i frammenti da 1-2 pixel senza erodere i muri reali.
            rawGrid = DenoiseGrid(rawGrid, 2);

            int width = rawGrid.GetLength(0);
            int height = rawGrid.GetLength(1);

            if (robotRadiusCells <= 0)
            {
                // Nessuna dilatazione: restituiamo una copia normalizzata (0 libero / 255 ostacolo)
                var passthrough = new int[width, height];
                for (int y = 0; y < height; y++)
                    for (int x = 0; x < width; x++)
                        passthrough[x, y] = rawGrid[x, y] > 200 ? 255 : 0;
                return passthrough;
            }

            // --- 1. DISTANCE TRANSFORM (Chamfer 3-4, due passate) ---
            // Scaliamo le distanze x10 così la diagonale (~1.41) usa il peso 14 e
            // l'ortogonale usa 10, coerente con i costi dell'A*. La distanza in celle
            // si ottiene poi dividendo per 10.
            const int ORTHO = 10;   // costo passo ortogonale
            const int DIAG = 14;    // costo passo diagonale
            const int INF = int.MaxValue / 4;

            var dist = new int[width, height];
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    dist[x, y] = rawGrid[x, y] > 200 ? 0 : INF;

            // Passata in avanti (top-left -> bottom-right)
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    int d = dist[x, y];
                    if (d == 0) continue;

                    if (x > 0) d = Math.Min(d, dist[x - 1, y] + ORTHO);
                    if (y > 0) d = Math.Min(d, dist[x, y - 1] + ORTHO);
                    if (x > 0 && y > 0) d = Math.Min(d, dist[x - 1, y - 1] + DIAG);
                    if (x < width - 1 && y > 0) d = Math.Min(d, dist[x + 1, y - 1] + DIAG);

                    dist[x, y] = d;
                }
            }

            // Passata all'indietro (bottom-right -> top-left)
            for (int y = height - 1; y >= 0; y--)
            {
                for (int x = width - 1; x >= 0; x--)
                {
                    int d = dist[x, y];
                    if (d == 0) continue;

                    if (x < width - 1) d = Math.Min(d, dist[x + 1, y] + ORTHO);
                    if (y < height - 1) d = Math.Min(d, dist[x, y + 1] + ORTHO);
                    if (x < width - 1 && y < height - 1) d = Math.Min(d, dist[x + 1, y + 1] + DIAG);
                    if (x > 0 && y < height - 1) d = Math.Min(d, dist[x - 1, y + 1] + DIAG);

                    dist[x, y] = d;
                }
            }

            // --- 2. APPLICAZIONE COSTI ---
            // Fascia di inflation pari a 2x il raggio del robot: abbastanza per ottenere
            // un decadimento graduale che tiene il robot lontano dai muri, senza chiudere
            // del tutto i corridoi (la versione precedente usava 3x ed era troppo aggressiva).
            double lethalDist = robotRadiusCells * 10.0;            // distanza scalata x10
            double inflationDist = robotRadiusCells * 2.0 * 10.0;   // fine della fascia sfumata
            double bandWidth = inflationDist - lethalDist;

            var inflatedGrid = new int[width, height];

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    int d = dist[x, y];

                    if (d <= lethalDist)
                    {
                        // Ostacolo originale o dentro il raggio rigido del robot -> non percorribile
                        inflatedGrid[x, y] = 255;
                    }
                    else if (d <= inflationDist && bandWidth > 0)
                    {
                        // Zona sfumata: costo decresce da 200 (bordo letale) a 0 (fine fascia)
                        double t = (d - lethalDist) / bandWidth; // 0..1
                        int cost = (int)Math.Round(200 * (1.0 - t));
                        inflatedGrid[x, y] = Math.Max(1, Math.Min(200, cost));
                    }
                    else
                    {
                        inflatedGrid[x, y] = 0; // Libero
                    }
                }
            }

            return inflatedGrid;
        }
    }
}
