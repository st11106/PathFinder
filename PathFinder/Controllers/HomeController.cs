using Microsoft.AspNetCore.Mvc;
using PathFinder.Models;
using PathFinder.Services;
using PathFinder.Strategies;
using System.Text.Json;

namespace PathFinder.Controllers
{
    public class HomeController : Controller
    {
        private readonly PathfindingService _pathfindingService;

        public HomeController(PathfindingService pathfindingService)
        {
            _pathfindingService = pathfindingService;
        }

        public IActionResult Index()
        {
            return View();
        }

        [HttpPost]
        public async Task<IActionResult> CalculatePath(
            IFormFile image,
            IFormFile metadataFile,
            string waypointsJson,
            int cellSize = 10,
            int robotRadius = 2,
            double? finalAngleDeg = null)
        {
            if (image == null || image.Length == 0) return BadRequest("Immagine planimetria mancante.");
            if (metadataFile == null || metadataFile.Length == 0) return BadRequest("File metadati mancante.");
            if (string.IsNullOrEmpty(waypointsJson)) return BadRequest("Punti mancanti.");
            if (cellSize <= 0) cellSize = 1;

            // 1. DESERIALIZZAZIONE JSON (Punti multi-tappa)
            // L'opzione CaseInsensitive risolve i problemi nel caso nel DTO le variabili si chiamino X e Y
            var options = new JsonSerializerOptions { PropertyNameCaseInsensitive = true };
            var inputPoints = JsonSerializer.Deserialize<List<PointDTO>>(waypointsJson, options);

            if (inputPoints == null || inputPoints.Count < 2)
                return BadRequest("Sono necessari almeno un punto di partenza e uno di arrivo.");

            // 2. LETTURA METADATI
            MapMetadata metadata;
            try
            {
                using var metaStream = metadataFile.OpenReadStream();
                metadata = await JsonSerializer.DeserializeAsync<MapMetadata>(metaStream);
                if (metadata == null) return BadRequest("Formato JSON metadati non valido.");
            }
            catch (Exception ex)
            {
                return BadRequest($"Errore metadati: {ex.Message}");
            }

            var transformer = new CoordinateTransformer(metadata);

            // 3. TRASFORMAZIONE DELL'ANGOLO FINALE
            double? canvasFinalAngleRad = null;
            if (finalAngleDeg.HasValue)
            {
                double amrFinalAngleRad = finalAngleDeg.Value * (Math.PI / 180.0);
                canvasFinalAngleRad = -amrFinalAngleRad;
            }

            // 4. CALCOLO PATH
            var targetCells = inputPoints
                .Select(p => new Coordinate(p.X / cellSize, p.Y / cellSize))
                .ToList();

            using var imageStream = image.OpenReadStream();
            var finalPoses = await _pathfindingService.CalculatePathAsync(
                imageStream,
                targetCells,
                cellSize,
                robotRadius,
                canvasFinalAngleRad);

            // 5. DOPPIO OUTPUT (Canvas + AMR)
            var finalResult = finalPoses.Select(p =>
            {
                double pixelX = (p.X * cellSize) + (cellSize / 2.0);
                double pixelY = (p.Y * cellSize) + (cellSize / 2.0);

                // Generiamo la posa calcolata rispetto la Planimetria reale 
                // Includiamo l'angolazione matematica dal modello Pose
                var amrPose = transformer.PixelsToAmrPose(pixelX, pixelY, p.Theta);

                // La Pose in 'finalPoses' ha già la correct 'speed' generata dall'HeadingCalculator
                // la preserviamo per il JSON.
                double actualSpeed = p.speed;

                return new
                {
                    canvas = new { x = pixelX, y = pixelY, theta = p.Theta },
                    amr = new { x = amrPose.X, y = amrPose.Y, theta = amrPose.Theta, speed = actualSpeed }
                };
            }).ToList();

            return Json(new { success = true, path = finalResult });
        }
    }
}
