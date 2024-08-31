﻿namespace PathFinder2D.PathFindingAlgorithms
{
    using PathFinder2D.DataStructures;
    using PathFinder2D.Managers;
    using PathFinder2D.UI;
    using System.Diagnostics;
    using System.Windows;
    using System.Windows.Media;

    /// <summary>
    /// A* algorithm.
    /// </summary>
    public class Astar
    {
        private readonly Graph graph;
        private readonly PathVisualizer pathVisualizer;
        private readonly PathVisualizerWPF pathVisualizerWPF;
        private Stopwatch aStarStopwatch;
        private int visitedNodes = 0;
        private bool pathFound;
        private double shortestPathCost = 0;
        private bool running = false;
        private bool testOn = false;

        /// <summary>
        /// Initializes a new instance of the <see cref="Astar"/> class.
        /// </summary>
        /// <param name="graph">The graph to be processed by the A* algorithm.</param>
        /// <param name="visualizer">The path visualizer to visualize the A* algorithm in the console.</param>
        public Astar(Graph graph, PathVisualizer visualizer)
        {
            this.graph = graph;
            this.pathVisualizer = visualizer;
            this.aStarStopwatch = new Stopwatch();
        }

        /// <summary>
        /// Finds the shortest path between two given nodes.
        /// </summary>
        /// <param name="start">The point where the path begins.</param>
        /// <param name="end">The point where the path ends.</param>
        /// <returns>Shortest path in a form of a list of nodes.</returns>
        public List<Node> FindShortestPath(Node start, Node end)
        {
            this.aStarStopwatch.Start();
            this.running = true;

            start.Cost = 0;
            var gscore = new Dictionary<Node, double>();
            var priorityQueue = new PriorityQueue<Node, double>();
            priorityQueue.Enqueue(start, 0);

            // Initialize all nodes with max cost
            foreach (var row in this.graph.Nodes)
            {
                foreach (var node in row)
                {
                    gscore[node] = double.MaxValue;
                }
            }

            gscore[start] = 0;

            while (priorityQueue.Count > 0 && this.running)
            {
                var currentNode = priorityQueue.Dequeue();

                if (currentNode.Visited)
                {
                    continue;
                }

                currentNode.Visited = true;
                this.visitedNodes++;

                this.pathVisualizer.VisualizePath(currentNode, start, end);

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
                    this.aStarStopwatch.Stop();

                    return finalPath;
                }

                foreach (var (neighborNode, cost) in this.graph.GetNeighborsWithCosts(currentNode))
                {
                    if (!this.running)
                    {
                        break;
                    }

                    if (neighborNode.Visited)
                    {
                        continue;
                    }

                    double tentative_g_score = gscore[currentNode] + cost;

                    if (tentative_g_score < gscore[neighborNode])
                    {
                        gscore[neighborNode] = tentative_g_score;
                        double priority = tentative_g_score + this.Heuristic(end, neighborNode);
                        neighborNode.Parent = currentNode;
                        priorityQueue.Enqueue(neighborNode, priority);
                    }
                }
            }

            this.running = false;
            this.aStarStopwatch.Stop();

            if (!this.testOn)
            {
                MessageBox.Show($"Path not found!");
            }

            // If the end node wasn't reached, return an empty path
            return new List<Node>();
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
        /// This method estimates how close a node is to the end point. It uses the Euclidean distance,
        /// which is just adding up the horizontal and vertical distances. This helps the algorithm
        /// decide which paths are worth looking at first to find the shortest route faster.
        /// </summary>
        /// <param name="end">The end point given by the user.</param>
        /// <param name="node">The node currently processed.</param>
        /// <returns>An estimated distance from the current node to the end point.</returns>
        private double Heuristic(Node end, Node node)
        {
            return Math.Sqrt(Math.Pow(end.X - node.X, 2) + Math.Pow(end.Y - node.Y, 2));
        }

        /// <summary>
        /// Retrieves the time A* took to find the end node.
        /// </summary>
        /// <returns>The time in milliseconds.</returns>
        public double GetStopwatchTime()
        {
            return this.aStarStopwatch.Elapsed.TotalMilliseconds;
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
        /// Retrieves the cost of the shortest path found by the A* algorithm.
        /// </summary>
        /// <returns>The cost of the shortest path.</returns>
        public double GetShortestPathCost()
        {
            return Math.Round(this.shortestPathCost, 1);
        }

        public bool IsRunning()
        {
            return this.running;
        }

        public void StopRunning()
        {
            this.running = false;
        }

        public void TurnOnTesting()
        {
            this.testOn = true;
        }
    }
}
