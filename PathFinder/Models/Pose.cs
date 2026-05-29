namespace PathFinder.Models
{
    // Usiamo un record per mantenere l'immutabilità e la leggerezza
    public readonly record struct Pose(double X, double Y, double Theta, double speed);
}