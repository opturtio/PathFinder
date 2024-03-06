namespace PathFinder.Managers
{
    using System.Diagnostics;
    using PathFinder.Algorithms;
    using PathFinder.DataStructures;

    /// <summary>
    /// Manages algorithm comparison.
    /// </summary>
    public class AlgorithmComparisonManager
    {
        private readonly string currentMap;
        private readonly string debugInput;
        private readonly Graph graph;
        private readonly CommandManager commandManager;
        private Dijkstra dijkstra;
        private Astar aStar;
        private JPS jps;
        private PathVisualizer pathVisualizer;
        private List<Node> shortestPathDijkstra;
        private List<Node> shortestPathAstar;
        private List<Node> shortestPathJps;
        private int shortestPathLengthDijkstra;
        private int shortestPathLengthAstar;
        private int shortestPathLengthJps;
        private OutputManager outputManager;

        /// <summary>
        /// Initializes a new instance of the <see cref="AlgorithmComparisonManager"/> class.
        /// </summary>
        /// <param name="comparisonGraph">The graph used to compare the Dijkstra and JPS algorithms.</param>
        /// /// <param name="currentMap">Current map in a string form.</param>
        public AlgorithmComparisonManager(Graph comparisonGraph, string currentMap)
        {
            this.graph = comparisonGraph;
            this.currentMap = currentMap;
            this.debugInput = debugInput;
            this.pathVisualizer = new PathVisualizer(this.graph, this.currentMap);
            this.shortestPathDijkstra = new List<Node>();
            this.shortestPathAstar = new List<Node>();
            this.shortestPathJps = new List<Node>();
            this.commandManager = new CommandManager();
            this.outputManager = new OutputManager();
        }

        /// <summary>
        /// Initializes the algorihms.
        /// </summary>
        public void Initialize()
        {
            var debugInput = this.commandManager.ProcessDebugOption();
            if (debugInput == "y")
            {
                this.pathVisualizer.ActivateDebugger();
            }
            else if (debugInput == "n")
            {
                this.pathVisualizer.DeactivateDebugger();
            }
            else
            {
                return;
            }

            var coords = PathCoordinatesValidator.StartValidation(this.graph, this.currentMap);

            this.jps = new JPS(this.graph, this.pathVisualizer);
            this.shortestPathJps = this.jps.FindShortestPath(this.graph.Nodes[coords[0]][coords[1]], this.graph.Nodes[coords[2]][coords[3]]);
            this.shortestPathLengthJps = ShortestPathBuilder.ShortestPathLength(this.graph.Nodes[coords[2]][coords[3]]);

            this.pathVisualizer.ClearVisitedNodes();
            this.graph.ResetNodes();

            this.aStar = new Astar(this.graph, this.pathVisualizer);
            this.shortestPathAstar = this.aStar.FindShortestPath(this.graph.Nodes[coords[0]][coords[1]], this.graph.Nodes[coords[2]][coords[3]]);
            this.shortestPathLengthAstar = ShortestPathBuilder.ShortestPathLength(this.graph.Nodes[coords[2]][coords[3]]);

            this.pathVisualizer.ClearVisitedNodes();
            this.graph.ResetNodes();

            this.dijkstra = new Dijkstra(this.graph, this.pathVisualizer);
            this.shortestPathDijkstra = this.dijkstra.FindShortestPath(this.graph.Nodes[coords[0]][coords[1]], this.graph.Nodes[coords[2]][coords[3]]);
            this.shortestPathLengthDijkstra = ShortestPathBuilder.ShortestPathLength(this.graph.Nodes[coords[2]][coords[3]]);

            
            if (this.jps.IsPathFound() && this.aStar.IsPathFound() && this.dijkstra.IsPathFound())
            {
                this.PrintResults();
            }
            else
            {
                Console.WriteLine("No paths found!\n");
            }
            
        }

        /// <summary>
        /// Prints to console the results of the Algorithm comparison.
        /// </summary>
        public void PrintResults()
        {
            var dijkstraMap = this.pathVisualizer.VisualizeShortestPath(this.shortestPathDijkstra);
            var aStarMap = this.pathVisualizer.VisualizeShortestPath(this.shortestPathAstar);
            var jpsMap = this.pathVisualizer.VisualizeShortestPath(this.shortestPathJps);

            Console.Clear();
            Console.WriteLine("Dijkstra shortest path:");
            Console.WriteLine(dijkstraMap);
            Console.WriteLine("A* shortest path:");
            Console.WriteLine(aStarMap);
            Console.WriteLine("JPS shortest path:");
            Console.WriteLine(jpsMap);

            Console.WriteLine("Results:");
            Console.WriteLine("--------------------------------------------------------------------");
            Console.WriteLine(String.Format("| {0,-11} | {1,-14} | {2,-20} | {3,-10} |", "Algorithm", "Visited nodes", "Time(milliseconds)", "Length"));
            Console.WriteLine(String.Format("| {0,-11} | {1,-14} | {2,-20} | {3,-10} |", "Dijkstra", this.dijkstra.GetVisitedNodes(), this.dijkstra.GetStopwatchTime(), this.shortestPathLengthDijkstra));
            Console.WriteLine(String.Format("| {0,-11} | {1,-14} | {2,-20} | {3,-10} |", "A*", this.aStar.GetVisitedNodes(), this.aStar.GetStopwatchTime(), this.shortestPathLengthAstar));
            Console.WriteLine(String.Format("| {0,-11} | {1,-14} | {2,-20} | {3,-10} |", "JPS", this.jps.GetVisitedNodes(), this.jps.GetStopwatchTime(), this.shortestPathLengthJps));
            Console.WriteLine("--------------------------------------------------------------------");

            Console.WriteLine();
        }
    }
}
