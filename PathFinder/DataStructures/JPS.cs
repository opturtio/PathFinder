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
        private double currentCost = 0;
        private PriorityQueue<Node, double> forcedNeighbors;
        private List<Node> forcedHelperList;

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
            this.forcedNeighbors = new PriorityQueue<Node, double>();
            this.forcedHelperList = new List<Node>();
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

            Node currentNode = null;

            while (successors.Count > 0 || this.forcedHelperList.Count > 0)
            {
                if (successors.Count > 0)
                {
                    currentNode = successors.Dequeue();
                    this.currentCost = currentNode.Cost;
                }
                else
                {
                    foreach (var forcedNode in this.forcedHelperList)
                    {
                        double newCost = this.currentCost + this.Heuristic(forcedNode, end);
                        forcedNode.Cost = newCost;
                        // Console.WriteLine(forcedNode.GetNodeInfo());
                        this.forcedNeighbors.Enqueue(forcedNode, newCost);
                    }

                    currentNode = this.forcedNeighbors.Dequeue();
                }

                this.visitedNodes++;

                if (currentNode == end || this.pathFound)
                {
                    this.pathFound = true;
                    break;
                }

                foreach (var direction in this.GetDirections())
                {
                    var jumpPoint = this.Jump(currentNode, direction, start, end, currentNode.Cost);

                    if (jumpPoint == null)
                    {
                        continue;
                    }

                    double newCost = currentNode.Cost + this.Heuristic(jumpPoint, end);

                    if (newCost < jumpPoint.Cost)
                    {
                        jumpPoint.Cost = newCost;

                        jumpPoint.JumpPoint = true;

                        successors.Enqueue(jumpPoint, newCost);
                    }
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
        private IEnumerable<(int x, int y)> GetDirections()
        {
            return new List<(int x, int y)>
            {
                (0, -1), (1, 0), (0, 1), (-1, 0),
                (1, -1), (1, 1), (-1, 1), (-1, -1),
            };
        }

        /// <summary>
        /// Recursive function to find a jump point in a specific direction.
        /// </summary>
        /// <param name="currentNode">The current node to jump from.</param>
        /// <param name="direction">The direction to jump in.</param>
        /// <param name="end">The end node of the pathfinding process.</param>
        /// <returns>The jump point node if one is found, otherwise null.</returns>
        private Node? Jump(Node currentNode, (int x, int y) direction, Node start, Node end, double currentCost)
        {
            int nextX = currentNode.X + direction.x;
            int nextY = currentNode.Y + direction.y;

            // Check if next position is within bounds and not an obstacle
            if (!this.graph.CanMove(nextX, nextY))
            {
                return null;
            }

            Node nextNode = this.graph.Nodes[nextY][nextX];

            if (nextNode.Visited)
            {
                return null;
            }

            nextNode.Visited = true;
            nextNode.Parent = currentNode;

            this.pathVisualizer.VisualizePath(nextNode, start, end, true);

            // If we've reached the end, return this node
            if (nextNode == end)
            {
                this.pathFound = true;
                return nextNode;
            }

            // For diagonal movement, check for forced neighbors along both axes
            if (direction.x != 0 && direction.y != 0)
            {
                if ((!this.graph.CanMove(nextNode.X + direction.x, nextNode.Y) && this.graph.CanMove(nextNode.X + direction.x, nextNode.Y + direction.y)) ||
                    (!this.graph.CanMove(nextNode.X, nextNode.Y + direction.y) && this.graph.CanMove(nextNode.X + direction.x, nextNode.Y + direction.y)))
                {
                    nextNode.Forced = true;
                    this.AddForcedNeighbor(nextNode);
                    return nextNode;
                }

                if (this.Jump(nextNode, (direction.x, 0), start, end, currentCost) != null || this.Jump(nextNode, (0, direction.y), start, end, currentCost) != null)
                {
                    return nextNode;
                }
            }
            else
            {
                if (direction.x != 0)
                {
                    if ((!this.graph.CanMove(nextNode.X, nextNode.Y + 1) && this.graph.CanMove(nextNode.X + direction.x, nextNode.Y + 1)) ||
                        (!this.graph.CanMove(nextNode.X, nextNode.Y - 1) && this.graph.CanMove(nextNode.X + direction.x, nextNode.Y - 1)))
                    {
                        nextNode.Forced = true;
                        this.AddForcedNeighbor(nextNode);
                        return nextNode;
                    }
                }
                else
                {
                    if ((!this.graph.CanMove(nextNode.X + 1, nextNode.Y) && this.graph.CanMove(nextNode.X + 1, nextNode.Y + direction.y)) ||
                        (!this.graph.CanMove(nextNode.X - 1, nextNode.Y) && this.graph.CanMove(nextNode.X - 1, nextNode.Y + direction.y)))
                    {
                        nextNode.Forced = true;
                        this.AddForcedNeighbor(nextNode);
                        return nextNode;
                    }
                }
            }

            if (currentCost < 10)
            {
                if (this.CheckIfGoal(nextNode, end))
                {
                    return nextNode;
                }
            }

            return this.Jump(nextNode, direction, start, end, currentCost);
        }

        private bool CheckIfGoal(Node nextNode, Node end)
        {
            int[] dx = new int[] { -1, 1, 0, 0, -1, -1, 1, 1 };
            int[] dy = new int[] { 0, 0, -1, 1, -1, 1, -1, 1 };

            for (int i = 0; i < 8; i++)
            {
                int newX = nextNode.X + dx[i];
                int newY = nextNode.Y + dy[i];

                if (newX == end.X && newY == end.Y)
                {
                    return true;
                }
            }

            return false;
        }

        private void AddForcedNeighbor(Node forced)
        {
            this.forcedHelperList.Add(forced);
            var neighbors = this.PruneNeighbors(forced);
            foreach (var neighbor in neighbors)
            {
                neighbor.Forced = true;
                this.forcedHelperList.Add(neighbor);
            }
        }

        private List<Node> PruneNeighbors(Node current)
        {
            List<Node> neighbors = new List<Node>();

            if (current.Parent != null)
            {
                int x = current.X;
                int y = current.Y;
                int px = current.Parent.X;
                int py = current.Parent.Y;

                int dx = (x - px) / Math.Max(Math.Abs(x - px), 1);
                int dy = (y - py) / Math.Max(Math.Abs(y - py), 1);

                // search diagonally
                if (dx != 0 && dy != 0)
                {
                    if (this.graph.CanMove(x, y + dy))
                    {
                        neighbors.Add(this.graph.Nodes[y + dy][x]);
                    }
                    if (this.graph.CanMove(x + dx, y))
                    {
                        neighbors.Add(this.graph.Nodes[y][x + dx]);
                    }
                    if (this.graph.CanMove(x + dx, y + dy))
                    {
                        neighbors.Add(this.graph.Nodes[y + dy][x + dx]);
                    }
                    if (!this.graph.CanMove(x - dx, y))
                    {
                        neighbors.Add(this.graph.Nodes[y + dy][x - dx]);
                    }
                    if (!this.graph.CanMove(x, y - dy))
                    {
                        neighbors.Add(this.graph.Nodes[y - dy][x + dx]);
                    }
                }

                // search horizontally/vertically
                else
                {
                    if (dx == 0)
                    {
                        if (this.graph.CanMove(x, y + dy))
                        {
                            neighbors.Add(this.graph.Nodes[y + dy][x]);
                        }
                        if (!this.graph.CanMove(x + 1, y))
                        {
                            neighbors.Add(this.graph.Nodes[y + dy][x + 1]);
                        }
                        if (!this.graph.CanMove(x - 1, y))
                        {
                            neighbors.Add(this.graph.Nodes[y + dy][x - 1]);
                        }
                    }
                    else
                    {
                        if (this.graph.CanMove(x + dx, y))
                        {
                            neighbors.Add(this.graph.Nodes[y][x + dx]);
                        }
                        if (!this.graph.CanMove(x, y + 1))
                        {
                            neighbors.Add(this.graph.Nodes[y + 1][x + dx]);
                        }
                        if (!this.graph.CanMove(x, y - 1))
                        {
                            neighbors.Add(this.graph.Nodes[y - 1][x + dx]);
                        }
                    }
                }
            }

            return neighbors;
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
            return Math.Sqrt(Math.Pow(end.X - jumpPoint.X, 2) + Math.Pow(end.Y - jumpPoint.Y, 2));
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