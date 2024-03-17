using System.Diagnostics;

namespace PathFinder.DataStructures
{
    /// <summary>
    /// Jump Point Search algorithm class.
    /// Implements the pathfinding logic using the Jump Point Search algorithm.
    /// </summary>
    public class JPS
    {
        private readonly Graph graph;
        private readonly PathVisualizer pathVisualizer;
        private Stopwatch jpsStopwatch;
        private int visitedNodes = 0;
        private bool pathFound = false;

        /// <summary>
        /// Initializes a new instance of the <see cref="JPS"/> class.
        /// </summary>
        /// <param name="graph">Graph on which pathfinding is performed.</param>
        /// <param name="visualizer">Visualizer for path visualization.</param>
        public JPS(Graph graph, PathVisualizer visualizer)
        {
            this.graph = graph;
            this.pathVisualizer = visualizer;
            this.jpsStopwatch = new Stopwatch();
        }

        /// <summary>
        /// Finds the shortest path between the start and end nodes using Jump Point Search algorithm.
        /// </summary>
        /// <param name="start">The start node of the path.</param>
        /// <param name="end">The end node of the path.</param>
        /// <returns>List of nodes representing the shortest path.</returns>
        public List<Node> FindShortestPath(Node start, Node end)
        {
            this.jpsStopwatch.Start();

            start.Cost = 0;

            var successors = new PriorityQueue<Node, double>();
            successors.Enqueue(start, start.Cost);

            while (successors.Count > 0)
            {
                var currentNode = successors.Dequeue();

                this.visitedNodes++;

                if (currentNode == end)
                {
                    this.pathFound = true;
                    break;
                }

                foreach (var direction in this.GetStraightMoves())
                {
                    var jumpPoint = this.JumpStraight(currentNode, direction, start, end);

                    if (jumpPoint == null)
                    {
                        continue;
                    }

                    double newCost = currentNode.Cost + this.Heuristic(jumpPoint, end);

                    jumpPoint.Cost = newCost;

                    jumpPoint.JumpPoint = true;

                    successors.Enqueue(jumpPoint, newCost);
                }

                foreach (var direction in this.GetDiagonalMoves())
                {
                    var jumpPoint = this.JumpDiagonal(currentNode, direction, start, end);

                    if (jumpPoint == null)
                    {
                        continue;
                    }

                    double newCost = currentNode.Cost + this.Heuristic(jumpPoint, end);

                    jumpPoint.Cost = newCost;

                    jumpPoint.JumpPoint = true;

                    successors.Enqueue(jumpPoint, newCost);
                }
            }

            this.jpsStopwatch.Stop();

            if (this.pathFound)
            {
                return ShortestPathBuilder.ShortestPath(end);
            }

            return new List<Node>();
        }

        /// <summary>
        /// Generates potential directions for movement..
        /// </summary>
        /// <returns>A list of tuples representing the possible directions for movement.</returns>
        private IEnumerable<(int y, int x)> GetStraightMoves()
        {
            return new List<(int y, int x)> { (-1, 0), (1, 0), (0, -1), (0, 1) };
        }

        private IEnumerable<(int y, int x)> GetDiagonalMoves()
        {
            return new List<(int y, int x)> { (-1, -1), (1, 1), (1, -1), (-1, 1) };
        }

        /// <summary>
        /// Recursive function to find a jump point in a specific direction.
        /// </summary>
        /// <param name="currentNode">The current node to jump from.</param>
        /// <param name="direction">The direction to jump in.</param>
        /// <param name="end">The end node of the pathfinding process.</param>
        /// <returns>The jump point node if one is found, otherwise null.</returns>
        private Node? JumpStraight(Node currentNode, (int y, int x) direction, Node start, Node end)
        {
            var distance = currentNode.Cost;
            int mapLength = this.graph.MapLength();
            int mapWidth = this.graph.MapWidth();

            while (true)
            {
                int nextX = currentNode.X + direction.x;
                int nextY = currentNode.Y + direction.y;
                distance++;

                // Check if next position is within bounds and not an obstacle
                if (!this.graph.CanMove(nextX, nextY))
                {
                    return null;
                }

                Node nextNode = this.graph.Nodes[nextY][nextX];

                if (nextNode.Cost != double.MaxValue && distance > nextNode.Cost)
                {
                    return null;
                }

                nextNode.Parent = this.graph.Nodes[nextNode.Y - direction.y][nextNode.X - direction.x];

                nextNode.Cost = distance;

                nextNode.Visited = true;

                this.pathVisualizer.VisualizePath(nextNode, start, end, true);

                // If we've reached the end, return this node. TARKASTA TÄMÄ
                if (currentNode == end)
                {
                    nextNode.Cost = distance;
                    return currentNode;
                }

                if (direction.y == 0)
                {
                    if ((nextNode.Y + 1) < mapLength && nextNode.X - direction.x >= 0 && nextNode.X - direction.x < mapWidth)
                    {
                        if (this.graph.Nodes[nextNode.Y + 1][nextNode.X - direction.x].IsObstacle)
                        {
                            return currentNode;
                        }
                    }

                    if (nextNode.Y - 1 >= 0 && nextNode.X - direction.x >= 0 && nextNode.X - direction.x < mapWidth)
                    {
                        if (this.graph.Nodes[nextNode.Y - 1][nextNode.X - direction.x].IsObstacle)
                        {
                            return currentNode;
                        }
                    }
                }
                else
                {
                    if ((nextNode.X + 1) < mapWidth && nextNode.Y - direction.y >= 0 && nextNode.Y - direction.y < mapLength)
                    {
                        if (this.graph.Nodes[nextNode.Y - direction.y][nextNode.X + 1].IsObstacle)
                        {
                            return currentNode;
                        }
                    }

                    if ((nextNode.X - 1) >= 0 && nextNode.Y - direction.y >= 0 && nextNode.Y - direction.y < mapLength)
                    {
                        if (this.graph.Nodes[nextNode.Y - direction.y][nextNode.X - 1].IsObstacle)
                        {
                            return currentNode;
                        }
                    }
                }

            }
        }

        private Node? JumpDiagonal(Node currentNode, (int y, int x) direction, Node start, Node end)
        {
            var distance = currentNode.Cost;

            while (true)
            {
                int nextX = currentNode.X + direction.x;
                int nextY = currentNode.Y + direction.y;
                distance += Math.Sqrt(2);

                // Check if next position is within bounds and not an obstacle
                if (!this.graph.CanMove(nextX, nextY))
                {
                    return null;
                }

                if (this.graph.Nodes[nextY - direction.y][nextX].IsObstacle || this.graph.Nodes[nextY][nextX - direction.x].IsObstacle)
                {
                    return null;
                }

                Node nextNode = this.graph.Nodes[nextY][nextX];

                if (nextNode.Cost != double.MaxValue && distance > nextNode.Cost)
                {
                    return null;
                }

                nextNode.Cost = distance;

                nextNode.Parent = this.graph.Nodes[nextNode.Y - direction.y][nextNode.X - direction.x];

                nextNode.Visited = true;

                this.pathVisualizer.VisualizePath(nextNode, start, end, true);

                // If we've reached the end, return this node. TARKASTA TÄMÄ
                if (currentNode == end)
                {
                    nextNode.Cost = distance;
                    return currentNode;
                }

                if (this.JumpStraight(currentNode, (direction.y, 0), start, end) != null)
                {
                    return currentNode;
                }

                if (this.JumpStraight(currentNode, (0, direction.x), start, end) != null)
                {
                    return currentNode;
                }
            }
        }

        /// <summary>
        /// This method estimates how close a node is to the end point. It uses the Euclidean distance,
        /// which is just adding up the horizontal and vertical distances. This helps the algorithm
        /// decide which paths are worth looking at first to find the shortest route faster.
        /// </summary>
        /// <param name="end">The end point given by the user.</param>
        /// <param name="neighborNode">The node currently processed.</param>
        /// <returns>An estimated distance from the current node to the end point.</returns>
        private double Heuristic(Node jumpPoint, Node end)
        {
            /*
            Console.WriteLine($"end.X: {end.X}");
            Console.WriteLine($"end.Y: {end.Y}");
            Console.WriteLine($"jumpPoint.X: {jumpPoint.X}");
            Console.WriteLine($"jumpPoint.Y: {jumpPoint.Y}");
            */
            double yDist = Math.Abs(jumpPoint.Y - end.Y);
            double xDist = Math.Abs(jumpPoint.X - end.X);

            return yDist + xDist + ((Math.Sqrt(2) - 2) * Math.Min(yDist, xDist));
        }

        /// <summary>
        /// Retrieves the total number of nodes that have been visited during the pathfinding.
        /// </summary>
        /// <returns>An integer representing the count of visited nodes.</returns>
        public int GetVisitedNodes()
        {
            return this.visitedNodes;
        }

        /// <summary>
        /// Retrieves the time JPS took to find the end node.
        /// </summary>
        /// <returns>The time in milliseconds.</returns>
        public double GetStopwatchTime()
        {
            return this.jpsStopwatch.Elapsed.TotalMilliseconds;
        }

        /// <summary>
        /// Determines whether a path from the start node to the end node has been found.
        /// </summary>
        /// <returns>A boolean value indicating whether a path was successfully found. Returns true if a path exists, otherwise false.</returns>
        public bool IsPathFound()
        {
            return this.pathFound;
        }
    }
}