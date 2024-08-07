﻿namespace PathFinder.Algorithms
{
    using PathFinder.DataStructures;
    using System.Diagnostics;
    using System.Net.Http.Headers;

    /// <summary>
    /// Dijkstra algorithm.
    /// </summary>
    public class Dijkstra
    {
        private readonly Graph graph;
        private readonly PathVisualizer pathVisualizer;
        private Stopwatch dijkstraStopwatch;
        private int shortestPathLength = 0;
        private int visitedNodes = 0;
        private bool pathFound = false;

        /// <summary>
        /// Initializes a new instance of the <see cref="Dijkstra"/> class.
        /// </summary>
        /// <param name="graph">The graph to be processed by the Dijkstra algorithm.</param>
        /// <param name="visualizer">The path visualizer to visualize the A* algorithm in the console.</param>
        public Dijkstra(Graph graph, PathVisualizer visualizer)
        {
            this.graph = graph;
            this.pathVisualizer = visualizer;
            this.dijkstraStopwatch = new Stopwatch();
        }

        /// <summary>
        /// Finds the shortest path between two given nodes.
        /// </summary>
        /// <param name="start">The point where the path begins.</param>
        /// <param name="end">The point where the path ends.</param>
        /// <returns>Shortest path in a form of a list of nodes.</returns>
        public List<Node> FindShortestPath(Node start, Node end)
        {
            this.dijkstraStopwatch.Start();

            start.Cost = 0;

            // Create a queue that sorts points by how far they are
            var priorityQueue = new PriorityQueue<Node, double>();
            priorityQueue.Enqueue(start, 0);

            while (priorityQueue.Count > 0)
            {
                // Selects the node with the shortest distance from the queue.
                var currentNode = priorityQueue.Dequeue();

                if (currentNode.Visited)
                {
                    continue;
                }

                currentNode.Visited = true;

                this.visitedNodes++;

                // Visualizes the path.
                this.pathVisualizer.VisualizePath(currentNode, start, end);

                if (currentNode == end)
                {
                    this.pathFound = true;
                    break;
                }

                // Check the nodes connected to the current point.
                foreach (var (neighborNode, cost) in this.graph.GetNeighborsWithCosts(currentNode))
                {
                    if (neighborNode.Visited)
                    {
                        continue;
                    }

                    double newCost = currentNode.Cost + cost;

                    // Update neighbor's distance and parent if a shorter path is found, then queue it for further exploration.
                    if (newCost < neighborNode.Cost)
                    {
                        neighborNode.Cost = newCost;
                        neighborNode.Parent = currentNode;
                        priorityQueue.Enqueue(neighborNode, newCost);
                    }
                }
            }

            this.dijkstraStopwatch.Stop();

            if (this.pathFound)
            {
                this.shortestPathLength = ShortestPathBuilder.ShortestPathLength(end);
                return ShortestPathBuilder.ShortestPath(end);
            }

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
        /// Retrieves the time Dijkstra took to find the end node.
        /// </summary>
        /// <returns>The time in milliseconds.</returns>
        public double GetStopwatchTime()
        {
            return this.dijkstraStopwatch.Elapsed.TotalMilliseconds;
        }

        /// <summary>
        /// Determines whether a path from the start node to the end node has been found.
        /// </summary>
        /// <returns>A boolean value indicating whether a path was successfully found. Returns true if a path exists, otherwise false.</returns>
        public bool IsPathFound()
        {
            return this.pathFound;
        }

        public int GetShortestPathLength()
        {
            return this.shortestPathLength;
        }
    }
}
