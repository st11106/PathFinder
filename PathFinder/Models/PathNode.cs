namespace PathFinder.Models
{
    public class PathNode
    {
        public Coordinate Position { get; }
        public int GCost { get; set; } = int.MaxValue;
        public int HCost { get; set; } = 0;
        public int FCost => GCost + HCost;
        public PathNode? Parent { get; set; }
        public bool InOpenList { get; set; } = false;

        public PathNode(Coordinate position)
        {
            Position = position;
        }
    }
}
