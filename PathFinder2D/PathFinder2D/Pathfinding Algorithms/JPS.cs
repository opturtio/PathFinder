﻿namespace PathFinder2D.PathFindingAlgorithms
{
    using PathFinder2D.DataStructures;
    using PathFinder2D.Managers;
    using PathFinder2D.UI;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Linq;
    using System.Windows;
    using System.Windows.Media;

    /// <summary>
    /// Jump Point Search (JPS) algorithm implementation.
    /// </summary>
    public class JPS
    {
        private readonly Graph graph;
        private readonly PathVisualizer pathVisualizer;
        private Stopwatch jpsStopwatch;
        private double shortestPathCost = 0;
        private int visitedNodes = 0;
        private bool pathFound = false;
        private bool running = false;
        private bool testOn = false;

        /// <summary>
        /// Initializes a new instance of the <see cref="JPS"/> class.
        /// </summary>
        /// <param name="graph">The graph to be processed by the JPS algorithm.</param>
        /// <param name="visualizer">The path visualizer to visualize the JPS algorithm in the console.</param>
        public JPS(Graph graph, PathVisualizer visualizer)
        {
            this.graph = graph;
            this.pathVisualizer = visualizer;
            this.jpsStopwatch = new Stopwatch();
        }

        /// <summary>
        /// Finds the shortest path between two given nodes using the JPS algorithm.
        /// </summary>
        /// <param name="start">The point where the path begins.</param>
        /// <param name="end">The point where the path ends.</param>
        /// <returns>A tuple containing the shortest path, number of operations, the total cost, and the list of visited nodes.</returns>
        public (List<Node> Path, int Operations, double Cost, List<Node> Visited) FindShortestPath(Node start, Node end)
        {
            this.jpsStopwatch.Start();
            this.running = true;

            start.Cost = 0;
            var gscore = new Dictionary<Node, double> { { start, 0 } };
            var openList = new BinaryHeap<Node>();
            openList.Insert(start);

            while (openList.Count > 0 && this.running)
            {
                var currentNode = openList.ExtractMin();
                this.visitedNodes++;

                if (currentNode == end)
                {
                    this.pathFound = true;
                    this.shortestPathCost = gscore[end];

                    var finalPath = ShortestPathBuilder.ReconstructPath(end);
                    for (int i = 1; i < finalPath.Count; i++)
                    {
                        var fromNode = finalPath[i - 1];
                        var toNode = finalPath[i];

                        ((PathVisualizerWPF)this.pathVisualizer).DrawLineBetweenNodes(fromNode, toNode, Brushes.Red);
                    }

                    this.running = false;
                    this.jpsStopwatch.Stop();

                    return (finalPath, this.visitedNodes, this.shortestPathCost, gscore.Keys.ToList());
                }

                var neighbors = this.PruneNeighbors(currentNode);

                foreach (var neighbor in neighbors)
                {
                    var jumpPointCoords = this.Jump(neighbor.X, neighbor.Y, currentNode.X, currentNode.Y, start, end);

                    if (jumpPointCoords == null)
                    {
                        continue;
                    }

                    var jumpPoint = this.graph.Nodes[jumpPointCoords.Value.y][jumpPointCoords.Value.x];
                    double tentative_g_score = gscore[currentNode] + this.Heuristic(currentNode, jumpPoint);

                    if (!gscore.ContainsKey(jumpPoint) || tentative_g_score < gscore[jumpPoint])
                    {
                        gscore[jumpPoint] = tentative_g_score;
                        jumpPoint.Parent = currentNode;
                        jumpPoint.Cost = tentative_g_score + this.Heuristic(jumpPoint, end);
                        jumpPoint.JumpPoint = true;

                        if (!openList.Contains(jumpPoint))
                        {
                            this.pathVisualizer.VisualizePath(jumpPoint, start, end, true);
                            openList.Insert(jumpPoint);
                        }
                    }
                }
            }

            this.running = false;
            this.jpsStopwatch.Stop();

            if (!this.testOn)
            {
                MessageBox.Show($"Path not found!");
            }

            return (new List<Node>(), this.visitedNodes, 0, gscore.Keys.ToList());
        }


        /// <summary>
        /// Determines the movement direction between two points.
        /// </summary>
        /// <param name="to">The destination coordinate.</param>
        /// <param name="from">The starting coordinate.</param>
        /// <returns>An integer representing the direction of movement: -1, 0, or 1.</returns>
        private int MovementDirection(int to, int from)
        {
            int direction = to - from;

            if (direction > 0)
            {
                return 1;
            }
            else if (direction < 0)
            {
                return -1;
            }

            return 0;
        }

        /// <summary>
        /// Prunes the neighbors of the current node to identify possible jump points.
        /// </summary>
        /// <param name="current">The current node being processed.</param>
        /// <returns>A list of neighboring nodes that are potential jump points.</returns>
        private List<Node> PruneNeighbors(Node current)
        {
            if (!this.running)
            {
                return new List<Node>();
            }

            List<Node> neighbors = new List<Node>();

            if (current.Parent != null)
            {
                int x = current.X;
                int y = current.Y;
                int px = current.Parent.X;
                int py = current.Parent.Y;

                int dx = this.MovementDirection(x, px);
                int dy = this.MovementDirection(y, py);

                if (dx != 0 && dy != 0)
                {
                    if (this.IsValidPosition(x, y + dy))
                    {
                        neighbors.Add(this.graph.Nodes[y + dy][x]);
                    }

                    if (this.IsValidPosition(x + dx, y))
                    {
                        neighbors.Add(this.graph.Nodes[y][x + dx]);
                    }

                    if (this.IsValidPosition(x + dx, y + dy))
                    {
                        neighbors.Add(this.graph.Nodes[y + dy][x + dx]);
                    }

                    if (!this.IsValidPosition(x - dx, y) && this.IsValidPosition(x - dx, y + dy))
                    {
                        neighbors.Add(this.graph.Nodes[y + dy][x - dx]);
                    }

                    if (!this.IsValidPosition(x, y - dy) && this.IsValidPosition(x + dx, y - dy))
                    {
                        neighbors.Add(this.graph.Nodes[y - dy][x + dx]);
                    }
                }
                else
                {
                    if (dx == 0)
                    {
                        if (this.IsValidPosition(x, y + dy))
                        {
                            neighbors.Add(this.graph.Nodes[y + dy][x]);
                        }

                        if (!this.IsValidPosition(x + 1, y) && this.IsValidPosition(x + 1, y + dy))
                        {
                            neighbors.Add(this.graph.Nodes[y + dy][x + 1]);
                        }

                        if (!this.IsValidPosition(x - 1, y) && this.IsValidPosition(x - 1, y + dy))
                        {
                            neighbors.Add(this.graph.Nodes[y + dy][x - 1]);
                        }
                    }
                    else
                    {
                        if (this.IsValidPosition(x + dx, y))
                        {
                            neighbors.Add(this.graph.Nodes[y][x + dx]);
                        }

                        if (!this.IsValidPosition(x, y + 1) && this.IsValidPosition(x + dx, y + 1))
                        {
                            neighbors.Add(this.graph.Nodes[y + 1][x + dx]);
                        }

                        if (!this.IsValidPosition(x, y - 1) && this.IsValidPosition(x + dx, y - 1))
                        {
                            neighbors.Add(this.graph.Nodes[y - 1][x + dx]);
                        }
                    }
                }
            }
            else
            {
                var neighborCosts = this.graph.GetNeighborsWithCosts(current);

                foreach (var (neighbor, cost) in neighborCosts)
                {
                    if (this.IsValidPosition(neighbor.X, neighbor.Y))
                    {
                        neighbors.Add(neighbor);
                    }
                }
            }

            return neighbors;
        }

        /// <summary>
        /// Checks whether a position is valid (within bounds and not an obstacle).
        /// </summary>
        /// <param name="x">The X-coordinate of the position.</param>
        /// <param name="y">The Y-coordinate of the position.</param>
        /// <returns>A boolean indicating if the position is valid.</returns>
        private bool IsValidPosition(int x, int y)
        {
            return y >= 0 && y < this.graph.Nodes.Count &&
                   x >= 0 && x < this.graph.Nodes[y].Count &&
                   !this.graph.Nodes[y][x].IsObstacle;
        }

        /// <summary>
        /// Performs the Jump function, which identifies the next jump point in a given direction.
        /// </summary>
        /// <param name="x">The current X-coordinate in the direction of movement.</param>
        /// <param name="y">The current Y-coordinate in the direction of movement.</param>
        /// <param name="px">The previous X-coordinate (from the parent node).</param>
        /// <param name="py">The previous Y-coordinate (from the parent node).</param>
        /// <param name="start">The start node of the path.</param>
        /// <param name="end">The end node of the path.</param>
        /// <returns>The coordinates of the jump point if found, otherwise null.</returns>
        private (int x, int y)? Jump(int x, int y, int px, int py, Node start, Node end)
        {
            int dx = x - px;
            int dy = y - py;

            if (!this.IsValidPosition(x, y) || !this.running)
            {
                return null;
            }

            Node currentNode = this.graph.Nodes[y][x];

            this.visitedNodes++;

            this.pathVisualizer.VisualizePath(currentNode, start, end, true);

            if (currentNode == end)
            {
                this.pathFound = true;
                return (x, y);
            }

            var forcedNeighbor = GetForcedNeighbors(x, y, dx, dy, start, end);

            if (forcedNeighbor != null)
            {
                return forcedNeighbor;
            }

            return this.Jump(x + dx, y + dy, x, y, start, end);
        }

        /// <summary>
        /// Checks for forced neighbors based on the current movement direction.
        /// </summary>
        /// <param name="x">The current X-coordinate in the grid.</param>
        /// <param name="y">The current Y-coordinate in the grid.</param>
        /// <param name="dx">The change in the X-coordinate from the parent node.</param>
        /// <param name="dy">The change in the Y-coordinate from the parent node.</param>
        /// <param name="start">The starting node of the path.</param>
        /// <param name="end">The target end node of the path.</param>
        /// <returns>A tuple containing the coordinates of a forced neighbor if one exists, otherwise returns null.</returns>
        private (int x, int y)? GetForcedNeighbors(int x, int y, int dx, int dy, Node start, Node end)
        {
            if (dx != 0 && dy != 0)
            {
                if ((this.IsValidPosition(x - dx, y + dy) && !this.IsValidPosition(x - dx, y)) ||
                    (this.IsValidPosition(x + dx, y - dy) && !this.IsValidPosition(x, y - dy)))
                {
                    return (x, y);
                }

                if (this.Jump(x + dx, y, x, y, start, end) != null || this.Jump(x, y + dy, x, y, start, end) != null)
                {
                    return (x, y);
                }
            }
            else
            {
                if (dx != 0)
                {
                    if ((this.IsValidPosition(x + dx, y + 1) && !this.IsValidPosition(x, y + 1)) ||
                        (this.IsValidPosition(x + dx, y - 1) && !this.IsValidPosition(x, y - 1)))
                    {
                        return (x, y);
                    }
                }
                else
                {
                    if ((this.IsValidPosition(x + 1, y + dy) && !this.IsValidPosition(x + 1, y)) ||
                        (this.IsValidPosition(x - 1, y + dy) && !this.IsValidPosition(x - 1, y)))
                    {
                        return (x, y);
                    }
                }
            }

            return null;
        }

        /// <summary>
        /// Estimates the distance between the current jump point and the end node using the Octile heuristic.
        /// </summary>
        /// <param name="jumpPoint">The current node being evaluated.</param>
        /// <param name="end">The end node.</param>
        /// <returns>The estimated cost from the current node to the goal node.</returns>
        private double Heuristic(Node jumpPoint, Node end)
        {
            int dx = Math.Abs(end.X - jumpPoint.X);
            int dy = Math.Abs(end.Y - jumpPoint.Y);
            double D = 1.0;
            double D2 = Math.Sqrt(2);

            return D * (dx + dy) + (D2 - 2 * D) * Math.Min(dx, dy);
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

        /// <summary>
        /// Retrieves the length of the shortest path found by the JPS algorithm.
        /// </summary>
        /// <returns>The length of the shortest path in number of nodes.</returns>
        public double GetShortestPathLength()
        {
            return Math.Round(this.shortestPathCost, 1);
        }

        /// <summary>
        /// Determines if the JPS algorithm is currently running.
        /// </summary>
        /// <returns>A boolean value indicating if the algorithm is running. Returns true if running, otherwise false.</returns>
        public bool IsRunning()
        {
            return this.running;
        }

        /// <summary>
        /// Stops the execution of the JPS algorithm.
        /// </summary>
        public void StopRunning()
        {
            this.running = false;
        }

        /// <summary>
        /// Enables testing mode to prevent certain UI interactions, such as message boxes.
        /// </summary>
        public void TurnOnTesting()
        {
            this.testOn = true;
        }
    }
}
