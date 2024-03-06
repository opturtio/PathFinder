namespace PathFinder.DataStructures
{
    using PathFinder.Managers;
    using System.Text;

    /// <summary>
    /// Visualizes the path finding process.
    /// </summary>
    public class PathVisualizer
    {
        private readonly string currentMapInitalized;
        private readonly Graph graph;
        private string currentMap;
        private bool isJps;
        private HashSet<Node> visitedNodes;
        private Node currentNode;
        private Node startNode;
        private Node endNode;
        private bool isDebug;

        /// <summary>
        /// Initializes a new instance of the <see cref="PathVisualizer"/> class.
        /// </summary>
        /// <param name="graph">The graph that to be visualized.</param>
        /// <param name="currentMap">The initial state of the map that will be visualized.</param>
        public PathVisualizer(Graph graph, string currentMap)
        {
            this.graph = graph;
            this.currentMap = currentMap;
            this.currentMapInitalized = currentMap;
            this.visitedNodes = new HashSet<Node>();
            this.isDebug = false;
        }

        public void ClearVisitedNodes()
        {
            this.visitedNodes.Clear();
        }

        /// <summary>
        /// Activates debugger so user can see how the algorithm works.
        /// </summary>
        public void ActivateDebugger()
        {
            this.isDebug = true;
        }

        /// <summary>
        /// Deactivates debugger.
        /// </summary>
        public void DeactivateDebugger()
        {
            this.isDebug = false;
        }

        /// <summary>
        /// Initializes the currentMap string.
        /// </summary>
        public void InitializeCurrentMap()
        {
            this.currentMap = this.currentMapInitalized;
        }

        /// <summary>
        /// Retrieves current map.
        /// </summary>
        /// <returns>Returns current map.</returns>
        public string GetCurrentMap()
        {
            return this.currentMap;
        }

        /// <summary>
        /// Visualizes the shortest path of the used algorithm.
        /// </summary>
        /// <param name="nodes">A list of nodes representing the shortest path from the start node to the end node.</param>
        public string VisualizeShortestPath(List<Node> nodes)
        {
            this.InitializeCurrentMap();
            this.visitedNodes.Clear();

            foreach (Node node in nodes)
            {
                // Add each node in the path to the visitedNodes list
                this.visitedNodes.Add(node);
            }

            // The last node in the list is the current node (end node of the path)
            this.currentNode = nodes.Last();

            // Generate the visualized map as a string
            return this.ShortestPathVisualizer();
        }

        /// <summary>
        /// Visualizes the current state of the path on the map in the console if the debugger is turned on.
        /// </summary>
        /// <param name="currentNode">The node currently being visited or processed by the algorithm.</param>
        public void VisualizePath(Node currentNode, Node start, Node end, bool jps = false)
        {
            if (this.isDebug)
            {
                this.currentNode = currentNode;
                this.startNode = start;
                this.endNode = end;
                this.isJps = jps;
                this.visitedNodes.Add(this.currentNode);
                this.Visualize();
            }
        }

        /// <summary>
        /// Visualizes the current and visited nodes on map state.
        /// </summary>
        public void Visualize()
        {
            string[] rows = this.currentMap.Split('\n');
            var outputBuffer = new StringBuilder();

            for (int y = 0; y < rows.Length; y++)
            {
                rows[y] = rows[y].Trim();

                for (int x = 0; x < rows[y].Length; x++)
                {
                    Node node = this.graph.Nodes[y][x];

                    if (node == this.currentNode)
                    {
                        outputBuffer.Append('X');
                    }
                    else if (node.Forced && node != this.endNode && this.isJps)
                    {
                        outputBuffer.Append('F');
                    }
                    else if (node.JumpPoint && node != this.endNode && this.isJps)
                    {
                        outputBuffer.Append('J');
                    }
                    else if (node == this.startNode)
                    {
                        outputBuffer.Append('S');
                    }
                    else if (node == this.endNode)
                    {
                        outputBuffer.Append('G');
                    }
                    else if (this.visitedNodes.Contains(node))
                    {
                        outputBuffer.Append('#');
                    }
                    else
                    {
                        outputBuffer.Append(rows[y][x]);
                    }
                }

                outputBuffer.AppendLine();
            }

            Console.SetCursorPosition(0, 0);
            Console.Write(outputBuffer.ToString());
            Thread.Sleep(30);
        }

        /// <summary>
        /// Visualizes the pathfinding process for debugging purposes.
        /// </summary>
        /// <param name="map">Empty string representation of the map.</param>
        /// <param name="jps">Indicates whether the visualization is for the Jump Point Search (JPS) algorithm.
        /// Set to true if the map is being visualized for JPS. Defaults to false for other pathfinding algorithms.</param>
        /// <returns>A string representing the map after the pathfinding process, with special characters denoting key nodes and states.</returns>
        public string DebugVisualize(string map, bool jps = false)
        {
            string[] rows = map.Split('\n');
            var outputBuffer = new StringBuilder();

            for (int y = 0; y < rows.Length; y++)
            {
                rows[y] = rows[y].Trim();

                for (int x = 0; x < rows[y].Length; x++)
                {
                    Node node = this.graph.Nodes[y][x];

                    if (node.Visited)
                    {
                        outputBuffer.Append('#');
                    }
                    else if (jps == true && node.Forced && node != this.endNode)
                    {
                        outputBuffer.Append('F');
                    }
                    else if (jps == true && node.JumpPoint && node != this.endNode)
                    {
                        outputBuffer.Append('J');
                    }
                    else if (node == this.startNode)
                    {
                        outputBuffer.Append('S');
                    }
                    else if (node == this.endNode)
                    {
                        outputBuffer.Append('G');
                    }
                    else
                    {
                        outputBuffer.Append(rows[y][x]);
                    }
                }

                outputBuffer.AppendLine();
            }

            return outputBuffer.ToString();
        }

        private string ShortestPathVisualizer()
        {
            StringBuilder visualizedMap = new StringBuilder();
            string[] rows = this.currentMap.Split('\n');

            for (int y = 0; y < rows.Length; y++)
            {
                // Cleans the row from carriage return if one occurs.
                // Otherwise, an index out of range occurs.
                rows[y] = rows[y].Trim();

                for (int x = 0; x < rows[y].Length; x++)
                {
                    Node node = this.graph.Nodes[y][x];

                    if (node == this.currentNode)
                    {
                        visualizedMap.Append("X");
                    }
                    else if (this.visitedNodes.Contains(node))
                    {
                        visualizedMap.Append("#");
                    }
                    else
                    {
                        visualizedMap.Append(rows[y][x]);
                    }
                }

                visualizedMap.AppendLine();
            }

            return visualizedMap.ToString();
        }
    }
}
