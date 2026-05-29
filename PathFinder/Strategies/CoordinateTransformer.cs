using PathFinder.Models;

namespace PathFinder.Strategies
{
    public class CoordinateTransformer
    {
        private readonly MapMetadata _meta;

        public CoordinateTransformer(MapMetadata meta)
        {
            _meta = meta;
        }

        public Pose PixelsToAmrPose(double pixelX, double pixelY, double thetaCanvas)
        {
            // 1. Trasformazione in metri e allineamento all'origine reale
            double amrX = (pixelX * _meta.resolution) + _meta.origin_x;

            // 2. Inversione dell'asse Y (il canvas va in giù, il robot va in su)
            double amrY = ((_meta.img_height_px - pixelY) * _meta.resolution) + _meta.origin_y;

            // 3. Inversione dell'angolo per compensare l'inversione della Y
            double amrTheta = -thetaCanvas;

            // La velocità sarà inserita dal Calculator dei posizionamenti per via del calcolo su costmap
            // Assegniamo qui un valore neutro se il transformer è richiamato per conversioni basiche
            return new Pose(amrX, amrY, amrTheta, 0.0);
        }
    }
}