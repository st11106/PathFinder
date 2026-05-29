namespace PathFinder.Interfaces
{
    public interface ISpeedEvaluator
    {
        double GetSpeedForCost(int cost);
        bool IsCostAllowed(int cost, int maxAllowedCost);
    }
}