using PathFinder.Interfaces;
using PathFinder.Strategies;

var builder = WebApplication.CreateBuilder(args);

// Add services to the container.
builder.Services.AddControllersWithViews();

builder.Services.AddScoped<IGridFactory, GridFactory>();
builder.Services.AddScoped<IPathfindingEngine, AStarEngine>();
builder.Services.AddScoped<ILineOfSightChecker, BresenhamLineOfSightChecker>();
builder.Services.AddScoped<IHeadingCalculator, StopAndTurnHeadingCalculator>();
builder.Services.AddScoped<IPathOptimizer, RaycastingPathOptimizer>();
builder.Services.AddScoped<PathFinder.Services.PathfindingService>();
builder.Services.AddSingleton<ISpeedEvaluator, CostmapSpeedEvaluator>();
builder.Services.AddTransient<ILineOfSightChecker, BresenhamLineOfSightChecker>();
builder.Services.AddTransient<IHeadingCalculator, StopAndTurnHeadingCalculator>();
var app = builder.Build();

// Configure the HTTP request pipeline.
if (!app.Environment.IsDevelopment())
{
    app.UseExceptionHandler("/Home/Error");
    // The default HSTS value is 30 days. You may want to change this for production scenarios, see https://aka.ms/aspnetcore-hsts.
    app.UseHsts();
}

app.UseHttpsRedirection();
app.UseRouting();

app.UseAuthorization();

app.MapStaticAssets();

app.MapControllerRoute(
    name: "default",
    pattern: "{controller=Home}/{action=Index}/{id?}")
    .WithStaticAssets();


app.Run();
