﻿using System;
using System.Collections.Generic;
using System.IO;

namespace PathFinder.Services
{
    /// <summary>
    /// Manages the loading of files from a specified directory.
    /// </summary>
    public class FileLoader
    {
        private readonly string _mapsDirectory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "..", "..", "..", "Resources", "Maps");
        private string map = "";

        /// <summary>
        /// Loads the names of map files from the directory at .Resources/Maps/.
        /// </summary>
        /// <returns>A list of strings, where each string is the name of a map file found in the directory.</returns>
        public List<string> LoadMapFileNames()
        {
            var mapFileNames = new List<string>();

            if (!Directory.Exists(_mapsDirectory))
            {
                Console.WriteLine("Maps directory not found!");
                return mapFileNames;
            }

            foreach (var filePath in Directory.GetFiles(_mapsDirectory))
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
            int indexNumber = int.Parse(indexNum)-1;
            var mapFileNames = LoadMapFileNames();
            if (indexNumber >= 0 && indexNumber <= mapFileNames.Count)
            {
                string selectedMapFilePath = Path.Combine(_mapsDirectory, mapFileNames[indexNumber]);
                map = File.ReadAllText(selectedMapFilePath);
            }
            else
                Console.WriteLine("Invalid map index number!");
            return map;
        }
    }
}