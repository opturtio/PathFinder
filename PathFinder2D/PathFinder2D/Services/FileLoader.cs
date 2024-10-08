﻿namespace PathFinder2D.Services
{
    using System.IO;

    /// <summary>
    /// Manages the loading of files from a specified directory.
    /// </summary>
    public class FileLoader
    {
        private string mapsDirectory;
        private string map = string.Empty;

        /// <summary>
        /// Initializes a new instance of the <see cref="FileLoader"/> class.
        /// </summary>
        public FileLoader()
        {
             this.mapsDirectory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "..", "..", "Resources", "Maps");
        }

        /// <summary>
        /// Loads the names of map files from the directory at .Resources/Maps/.
        /// </summary>
        /// <returns>A list of strings, where each string is the name of a map file found in the directory.</returns>
        public List<string> LoadMapFileNames()
        {
            var mapFileNames = new List<string>();

            if (!Directory.Exists(this.mapsDirectory))
            {
                Console.WriteLine($"Maps directory not found at: {this.mapsDirectory}");
                return mapFileNames;
            }

            foreach (var filePath in Directory.GetFiles(this.mapsDirectory))
            {
                string fileName = Path.GetFileName(filePath);
                mapFileNames.Add(fileName);
            }

            return mapFileNames;
        }

        /// <summary>
        /// Loads the content of a map file based on the specified index number.
        /// </summary>
        /// <param name="indexNum">The index number of the map file to load.</param>
        /// <returns>The content of the selected map file as a string.</returns>
        public string LoadMap(string indexNum)
        {
            try
            {
                int indexNumber = int.Parse(indexNum) - 1;
                var mapFileNames = this.LoadMapFileNames();
                if (indexNumber >= 0 && indexNumber <= mapFileNames.Count)
                {
                    string selectedMapFilePath = Path.Combine(this.mapsDirectory, mapFileNames[indexNumber]);
                    this.map = File.ReadAllText(selectedMapFilePath);
                }
                else
                    Console.WriteLine("Invalid map index number!");

                string[] rows = this.map.Split('\n');

                // Cleans the rows from carriage returns.
                for (int y = 0; y < rows.Length; y++)
                {
                    rows[y] = rows[y].Trim();
                }

                return this.map;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error loading map: {ex.Message}");
                return string.Empty;
            }
        }

        /// <summary>
        /// Loads the content of a map file by its name and converts it to a string.
        /// </summary>
        /// <param name="mapName">The name of the map file.</param>
        /// <returns>The content of the map file as a string, or an empty string if the file is not found or an error occurs.</returns>
        public string LoadMapByName(string mapName)
        {
            try
            {
                string mapFilePath = Path.Combine(this.mapsDirectory, mapName);

                if (File.Exists(mapFilePath))
                {
                    this.map = File.ReadAllText(mapFilePath);

                    string[] rows = this.map.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

                    if (rows.Length >= 5)
                    {
                        string heightLine = rows[1];
                        string widthLine = rows[2];

                        int height = int.Parse(heightLine.Split(' ')[1]);
                        int width = int.Parse(widthLine.Split(' ')[1]);

                        string[] mapData = rows.Skip(4).ToArray();

                        this.map = string.Join("\n", mapData);

                        return this.map;
                    }
                    else
                    {
                        Console.WriteLine($"Map file '{mapName}' is not in the expected format.");
                        return string.Empty;
                    }
                }
                else
                {
                    Console.WriteLine($"Map file '{mapName}' not found in directory '{this.mapsDirectory}'.");
                    return string.Empty;
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error loading map by name: {ex.Message}");
                return string.Empty;
            }
        }


        /// <summary>
        /// Sets the maps directory path. This method is used for testing.
        /// </summary>
        /// <param name="newPath">The new path for the map directory.</param>
        public void SetMapsDirectoryPath(string newPath)
        {
            this.mapsDirectory = newPath;
        }

        /// <summary>
        /// Gets the path of the map directory.
        /// </summary>
        /// <returns>The path to the directory where map files are stored.</returns>
        public string ReturnDirectoryPath()
        {
            return this.mapsDirectory;
        }
    }
}
