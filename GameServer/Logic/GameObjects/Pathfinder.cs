using LeagueSandbox.GameServer.Core.Logic;
using LeagueSandbox.GameServer.Logic.Maps;
using LeagueSandbox.GameServer.Logic.RAF;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

using LeagueSandbox.GameServer;

namespace LeagueSandbox.GameServer.Logic.GameObjects
{
    class Pathfinder
    {
        protected static Map chart;
        private const int GRID_SIZE = 1024;
        protected static int successes = 0, oot = 0, empties = 0;
        protected static int totalDuration = 0, durations = 0;
        protected static DateTime g_Clock = System.DateTime.Now;
        protected static bool debugOutput = false;
        protected static int MAX_PATHFIND_TRIES = 1000;

        public Pathfinder()/*:mesh(0),chart(0)*/ { }

        public static Path getPath(Vector2 from, Vector2 to, float boxSize)
        {

            Path path = new Path();
            PathJob job = new PathJob();

            if ((System.DateTime.Now - g_Clock).Milliseconds > 4000 && (successes + oot + empties) > 0)
                Logger.LogCoreInfo("Pathfinding successrate: " + (((float)successes / (float)(successes + oot + empties)) * (100.0f)));

            if (debugOutput)
                Logger.LogCoreInfo("Recording this minion movement.");

            if (chart == null) Logger.LogCoreError("Tried to find a path without setting the map.");
            if (getMesh() == null) Logger.LogCoreError("Can't start pathfinding without initialising the AIMesh");


            job.start = job.fromPositionToGrid(from); // Save start in grid info
            job.destination = job.fromPositionToGrid(to); // Save destination in grid info

            if (debugOutput)
            {
                Logger.LogCoreInfo("Going from (" + job.start.X + ", " + job.start.Y + ") to (" + job.destination.X + ", " + job.destination.Y);
            }

            job.insertObstructions(chart, getMesh()); // Ready the map.

            job.addToOpenList(job.start, null); // Let's start at the start.

            int tries;
            for (tries = 0; job.openList.Count != 0; tries++) // Go through the open list while it's not empty
            {
                if (debugOutput)
                    Logger.LogCoreInfo("Going through openlist. Tries: " + tries + " | Objects on list: " + job.openList.Count);


                if (tries == MAX_PATHFIND_TRIES)
                {
                  //  LeagueSandbox.GameDeubgger.getInstance().PingAtLocation(job.fromGridToPosition(job.start), Core.Logic.PacketHandlers.Packets.Pings.Ping_Assist);
                  //  LeagueSandbox.GameDeubgger.getInstance().PingAtLocation(job.fromGridToPosition(job.destination), Core.Logic.PacketHandlers.Packets.Pings.Ping_Assist);

                    path.error = PathError.PATH_ERROR_OUT_OF_TRIES;
                    oot++;
                    //CORE_WARNING("PATH_ERROR_OUT_OF_TRIES");
                    List<Vector2i> rawPath = job.reconstructUnfinishedPath();

                    job.cleanPath(rawPath);
                    path.waypoints = job.pathToPosition(rawPath); 

                    job.cleanLists();
                    return path;
                }
                else if (job.traverseOpenList(tries == 0))
                {
                    path.error = PathError.PATH_ERROR_NONE;
                    successes++;
                    //CORE_INFO("We finished a path.");

                    List<Vector2i> rawPath = job.reconstructPath();

                    job.cleanPath(rawPath);
                    path.waypoints = job.pathToPosition(rawPath);
               
                    job.cleanLists();
                    return path;
                }
            }

            if (debugOutput)
                Logger.LogCoreInfo("Going through openlist. Tries: " + tries + " | Objects on list: " + job.openList.Count);

            //OpenList is now empty, most likely caused by searching for a path from invalid location
            Logger.LogCoreWarning("PATH_ERROR_OPENLIST_EMPTY from:" + job.start + " to:" + job.destination);
            path.error = PathError.PATH_ERROR_OPENLIST_EMPTY;
            empties++;
            path.waypoints = new List<Vector2>();
            path.waypoints.Add(from);

            job.cleanLists();
            return path;
        }
        public static Path getPath(Vector2 from, Vector2 to)// { if (!chart->getAIMesh()) CORE_FATAL("Can't get path because of a missing AIMesh."); return getPath(from, to, PATH_DEFAULT_BOX_SIZE(mesh->getSize())); }
        {
            if (chart.getAIMesh() == null)
            {
                Logger.LogCoreError("Can't get path because of a missing AIMesh.");
            }
            return getPath(from, to, PATH_DEFAULT_BOX_SIZE(getMesh().getSize()));
        }
        public static void setMap(Map map)// { chart = map; mesh = chart->getAIMesh(); }
        {
            chart = map;
        }
        public static AIMesh getMesh()
        {
            if (chart == null)
            {
                Logger.LogCoreError("The map hasn't been set but the mesh was requested.");
                return null;
            }
            return chart.getAIMesh();
        }
        private static float PATH_DEFAULT_BOX_SIZE(float map_size)
        {
            return map_size / (float)GRID_SIZE;
        }
        /// <summary>
        /// Returns all occupied points in area. (Used for debug)
        /// </summary>
        /// <param name="position"></param>
        /// <returns></returns>
        public static List<Vector2> GetOccupiedPoints(Vector2 position)
        {
            PathJob job = new PathJob();
            job.insertObstructions(chart, getMesh()); // Ready the map.

            const float step = 60;
            const int stepCount = 30;

            List<Vector2> pointsToTest = new List<Vector2>();

            Vector2 start = new Vector2(position.X - (stepCount / 2) * step, position.Y - (stepCount / 2));
            //generate points for test
            for (int i=0;i<stepCount;i++)
                for(int j=0;j<stepCount;j++)
                {
                    Vector2 point = start + new Vector2(i * step, j * step);
                    pointsToTest.Add(point);
                }

            List<Vector2> result = new List<Vector2>();
            foreach (Vector2 point in pointsToTest)
            {
                Vector2 transformed = job.fromPositionToGrid(point);
                if (job.isGridNodeOccupied(transformed))
                    result.Add(point);

            }
            return result;
        }

    }
    class Path
    {
        public PathError error = PathError.PATH_ERROR_NONE;
        public List<Vector2> waypoints;

        public bool isPathed()
        {
            return error == PathError.PATH_ERROR_NONE;
        }
        public PathError getError()
        {
            return error;
        }
        //std::vector<Vector2> getWaypoints() { return waypoints; }
        public List<Vector2> getWaypoints()
        {
            return waypoints;
        }
    };

    class PathJob
    {
        private static int GRID_SIZE = 1024;
        private static int GRID_WIDTH = GRID_SIZE;
        private static int GRID_HEIGHT = GRID_SIZE;
        public List<PathNode> openList, closedList;
        public Grid[,] map = new Grid[GRID_WIDTH, GRID_HEIGHT];
        public Vector2 start, destination;

        public PathJob()
        {
            openList = new List<PathNode>();
            closedList = new List<PathNode>();
            start = new Vector2();
            destination = new Vector2();
            for (var i = 0; i < GRID_WIDTH; i++)
                for (var j = 0; j < GRID_HEIGHT; j++)
                    map[i, j] = new Grid();
        }

        public Vector2 fromGridToPosition(Vector2 position)
        {
            AIMesh mesh = Pathfinder.getMesh();
            if (mesh == null)
            {
                Logger.LogCoreError("Tried to get a grid location without an initialised AIMesh!");
                return new Vector2();
            }

            return position * PATH_DEFAULT_BOX_SIZE(mesh.getSize());
        }

        public Vector2 fromPositionToGrid(Vector2 position)
        {
            AIMesh mesh = Pathfinder.getMesh();
            if (mesh == null)
            {
                Logger.LogCoreError("Tried to get a position without an initialised AIMesh!");
                return new Vector2();
            }

            return (position / (float)PATH_DEFAULT_BOX_SIZE(mesh.getSize()));
        }


        private List<Vector2i> reconstructPathFromNode(PathNode last)
        {

            List<Vector2i> ret = new List<Vector2i>();
            if (last == null)
            {
                Logger.LogCoreWarning("Tried to reconstruct path from invalid node!");
                return ret;
            }

            //Note: last node of closedList is either destination node, or the closest node by 

            //go through node parents back to the first node
            do
            {
                ret.Add(new Vector2i(last.x, last.y));
                last = last.parent;
            } while (last.parent != null);

            ret.Reverse();

            return ret;
        }

        /// <summary>
        /// Reconstruct path from last closed node back to the origin.
        /// </summary>
        /// <returns>List of waypoint coordinates</returns>
        public List<Vector2i> reconstructPath()
        {
            if(closedList.Count()==0)
            {
                Logger.LogCoreWarning("Tried to reconstruct path from 0 nodes!");
                return new List<Vector2i>();
            }
            else
                return reconstructPathFromNode(closedList.Last());
        }
           

        public List<Vector2i> reconstructUnfinishedPath()
        {
            //find node with best score (most likely to be closest to target)

            PathNode pathnode = null;
            var lowestScore = 9e7;

            foreach (PathNode node in closedList)
             {
                 if (node.g+node.h < lowestScore)
                 {
                     lowestScore = node.g+node.h;
                     pathnode = node;
                 }
             }
            
             return reconstructPathFromNode(pathnode);
           
        }

        /// <summary>
        /// Cleans a path, removing unneeded waypoints
        /// </summary>
        /// <param name="path">list of grid coordinates</param>
        public void cleanPath(List<Vector2i> path)
        {
            return;

          if (path.Count() < 2) return;

          int startSize = path.Count();
          //CORE_WARNING("Cleaning path.. Current size is %d", startSize);

          int dirX = 0, dirY = 0;
          int lastIndex = 0;
          var prevPoint = path[lastIndex];
         

          for (var currentPoint = path[++lastIndex]; lastIndex<path.Count(); )
          {
              //Is waypoint in the same direction?
              if ((currentPoint.x - prevPoint.x == dirX) &&
                 (currentPoint.y - prevPoint.y == dirY))
              {
                    path.Remove(prevPoint);
                 // path.waypoints.erase(prevPoint);
                 // CORE_WARNING("Erased a waypoint");
              }
              else
              {
                  //Update direction change
                  dirX = currentPoint.x - prevPoint.x;
                  dirY = currentPoint.y - prevPoint.y;
              }

              prevPoint = currentPoint;
          }

         // CORE_WARNING("Done cleaning. New size is %d", path.waypoints.size());
         if (startSize != path.Count())
            Logger.LogCoreWarning("Removed %d nodes"+ (startSize - path.Count()));


            /* if (path.waypoints.size() < 2) return;
             int startSize = path.waypoints.size();
             CORE_WARNING("Cleaning path.. Current size is %d", startSize);

             int dirX = 0, dirY = 0;
             auto prevPoint = path.waypoints.begin();
             for (auto i = path.waypoints.begin() + 1; i != path.waypoints.end(); i++)
             {
                 if (((*i).X - (*prevPoint).X == dirX) &&
                    ((*i).Y - (*prevPoint).Y == dirY))
                 {
                     path.waypoints.erase(prevPoint);
                     CORE_WARNING("Erased a waypoint");
                 }
                 else
                 {
                     dirX = ((*i).X - (*prevPoint).X);
                     dirY = ((*i).Y - (*prevPoint).Y);
                 }

                 prevPoint = i;
             }

             CORE_WARNING("Done cleaning. New size is %d", path.waypoints.size());
             if (startSize != path.waypoints.size())
                 CORE_WARNING("Removed %d nodes", startSize - path.waypoints.size());*/
        }
        public bool traverseOpenList(bool first)
        {
            if (openList.Count == 0)
            {
                return false;
            }

            // This sorts every iteration, which means that everything but the last couple of elements are sorted.
            // TODO: That means, this can probably be optimised. Sort only the last elements and add them into the vector where they belong.
            // But honestly, it's running pretty fast so why bother
            openList.Sort((a, b) => (b.g + b.h).CompareTo(a.g + a.h));


            PathNode currentNode = openList.Last();
            openList.RemoveAt(openList.Count - 1);


            //DEBUG ping at this node
          //  LeagueSandbox.GameDeubgger.getInstance().PingAtLocation(fromGridToPosition(new Vector2(currentNode.x, currentNode.y)), Core.Logic.PacketHandlers.Packets.Pings.Ping_Assist);

            bool atDestination = (Math.Abs(currentNode.x - (int)destination.X) <= 1 && Math.Abs(currentNode.y - (int)destination.Y) <= 1);

            if (!atDestination) // While we're not there
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    if (currentNode.x + dx >= 0 && currentNode.x + dx < GRID_WIDTH) // Search in 8 directions, but we're supposed to stay in map
                    {
                        for (int dy = -1; dy <= 1; dy++)
                        {
                            if (!(dx == 0 && dy == 0)) // in all 8 directions, ignore the x==y==0 where we dont move
                            {
                                if (!isGridNodeOccupied(currentNode.x + dx, currentNode.y + dy)) // Is something here?
                                {
                                    PathNode conflictingNode = isNodeOpen(currentNode.x + dx, currentNode.y + dy); // Nothing is here, did we already add this to the open list?
                                    if (conflictingNode == null) // We did not, add it
                                    {
                                        addToOpenList(new Vector2(currentNode.x + dx, currentNode.y + dy), currentNode);
                                    }
                                    else if (conflictingNode.g > CALC_G(currentNode.g)) // I found a shorter route to this node.
                                    {
                                        conflictingNode.setParent(currentNode); // Give it a new parent
                                        conflictingNode.setScore((int)CALC_H(conflictingNode.x, conflictingNode.y, destination.X, destination.Y), (int)CALC_G(currentNode.g)); // set the new score.
                                    }
                                }
                            }
                        }
                    }
                }
            }


            closedList.Add(currentNode);
            return atDestination;
        }

        /// <summary>
        /// Calculates squared distance between two points
        /// </summary>
        /// <returns>Squared distance</returns>
        public float CALC_H(float CURX, float CURY, float ENDX, float ENDY)
        {
            return Math.Abs(CURX - ENDX) + Math.Abs(CURY - ENDY);
        }
        /// <summary>
        /// Calculates grid distance
        /// (Parent's distance + 1)
        /// </summary>
        /// <param name="PARENT_G">grid distance of parent node</param>
        /// <returns></returns>
        public float CALC_G(float PARENT_G)
        {
            return PARENT_G + 1;
        }

        public void addRealPosToOpenList(Vector2 position, PathNode parent)
        {
            addGridPosToOpenList(fromPositionToGrid(position), parent);
        }

        public void addGridPosToOpenList(Vector2 position, PathNode parent)
        {
            openList.Add(new PathNode(position, (int)CALC_G((parent != null) ? (parent.g) : (0)), (int)CALC_H(position.X, position.Y, destination.X, destination.Y), parent));
        }

        public void addToOpenList(Vector2 position, PathNode parent)
        {
            addGridPosToOpenList(position, parent);
        }

        public bool isGridNodeOccupied(Vector2 pos)
        {
            return isGridNodeOccupied((int)pos.X, (int)pos.Y);
        }
        public bool isGridNodeOccupied(int x, int y)
        {
            if ((x >= 0 && x < GRID_SIZE) && (y >= 0 && y < GRID_SIZE))
            {
                return map[x, y].isOccupied();
            }
            else return true;
        }
        public PathNode isNodeOpen(int x, int y)
        {
            // TODO: Optimise? This is where the application is residing in 96% of the time during pathfinding.

            // It checks if we've already added this x and y to the openlist. If we did, return it. 
            foreach (var i in openList)
            {
                if (i.x == x && i.y == y)
                    return i;
            }

            return null;
        }

        public void cleanLists()
        {
            openList.Clear();
            closedList.Clear();
        }
        public void insertObstructions(Map chart, AIMesh mesh)
        {
            if (mesh != null)
            {
                // Now to draw the mesh onto the thing.
                if (mesh.isLoaded()) // if we have loaded the mesh
                    for (int x = 0; x < GRID_WIDTH; x++) // for every grid piece
                        for (int y = 0; y < GRID_HEIGHT; y++)
                        {
                            Vector2 translated = fromGridToPosition(new Vector2(x, y));
                            if (!mesh.isWalkable(translated.X, translated.Y)) // If there's nothing at this position
                                map[x, y].occupied = true; // This is obstructed   

                        }
            }

            if (chart != null)
            {
                var objects = chart.getObjects();
                foreach (var i in objects) // For every object
                {
                    if (!(i.Value is Minion) && !(i.Value is Champion))
                        continue;

                    Vector2 gridPos = fromPositionToGrid(i.Value.getPosition()); // get the position in grid size

                    int radius = ((int)Math.Ceiling((float)i.Value.getCollisionRadius() / (float)PATH_DEFAULT_BOX_SIZE(mesh.getSize()))) / 2; // How many boxes does the radius of this object cover?

                    for (int dx = -radius; dx < radius; dx++) // For the whole radius in the width
                        if (gridPos.X + dx >= 0 && gridPos.X + dx < GRID_WIDTH) // As long as we're in the map (x)
                            for (int dy = -radius; dy < radius; dy++) // for the whole radius in the y
                                if (gridPos.Y + dy >= 0 && gridPos.Y + dy < GRID_HEIGHT) // As long as we're in the map (y)
                                    map[(int)gridPos.X + dx, (int)gridPos.Y + dy].occupied = true; // Occupy this piece of the map.
                }
            }

            /*  if (debugOutput())
              {
                  auto width = GRID_WIDTH;
                  auto height = GRID_HEIGHT;
  #define MIN(a,b) (((a)>(b))?(b):(a))
  #define MAX(a,b) (((a)>(b))?(a):(b))
                  std::ofstream imageFile("..\\..\\test.tga", std::ios::out | std::ios::binary);
                  if (!imageFile) return;

                  // The image header
                  unsigned char header[18] = { 0 };
                  header[2] = 1;  // truecolor
                  header[12] = width & 0xFF;
                  header[13] = (width >> 8) & 0xFF;
                  header[14] = height & 0xFF;
                  header[15] = (height >> 8) & 0xFF;
                  header[16] = 24;  // bits per pixel

                  imageFile.write((const char*)header, 18);

                  //for (int y = 0; y < height; y++)
                  for (int y = height - 1; y >= 0; y--)
                      for (int x = 0; x < width; x++)
                      {
                          // blue
                          imageFile.put(map[x][y].occupied * 128);
                          // green 
                          imageFile.put(map[x][y].occupied * 128);
                          // red 
                          imageFile.put(map[x][y].occupied * 128);
                      }

                  // The file footer. This part is totally optional.
                  static const char footer[26] =
                      "\0\0\0\0"  // no extension area

              "\0\0\0\0"  // no developer directory

              "TRUEVISION-XFILE"  // Yep, this is a TGA file

              ".";
                  imageFile.write(footer, 26);

                  imageFile.close();
              }*/
        }
        private static float PATH_DEFAULT_BOX_SIZE(float map_size)
        {
            return map_size / (float)GRID_SIZE;
        }

        public List<Vector2> pathToPosition(List<Vector2i> rawPath)
        {
            List<Vector2> ret = new List<Vector2>();
            foreach (Vector2i gridPos in rawPath)
            {
                ret.Add(fromGridToPosition(new Vector2(gridPos.x, gridPos.y)));
            }
            return ret;
        }

    }
    class Grid
    {
        public bool occupied;
        public bool isOccupied()
        {
            return occupied;/*return (occupant != NULL);*/
        }
        //Object * occupant;
    }
    class PathNode
    {
        public int x, y, h, g;
        public PathNode parent;
        private static int tableInitialised;
        private const int TABLE_SIZE = (2 << 15);
        private static List<PathNode> nodeTable = new List<PathNode>();

        public PathNode()
        {
            InitTable();
        }

        public PathNode(int ax, int ay, int ag, int ah, PathNode p)
        {
            Init(ax, ay, ag, ah, p);
        }

        public PathNode(Vector2 pos, int ag, int ah, PathNode p)
        {
            Init((int)pos.X, (int)pos.Y, ag, ah, p);
        }

        public void Init(int ax, int ay, int ag, int ah, PathNode p)
        {
            InitTable();
            x = ax;
            y = ay;
            h = ah;
            g = ag;
            parent = p;
        }

        public void setScore(int ah, int ag)
        {
            g = ag;
            h = ah;
        }

        public void setParent(PathNode p)
        {
            parent = p;
        }

        public static void DestroyTable()
        {
            tableInitialised = 2;
            nodeTable.Clear();
            tableInitialised = -1;
        }
        public static bool isTableEmpty()
        {
            InitTable();
            return nodeTable.Count == 0;
        }
        public static int missingNodes()
        {
            InitTable();
            return TABLE_SIZE - nodeTable.Capacity;
        }

        private static void InitTable()
        {
            if (tableInitialised != -1) return; // We have already initialised it or we're busy doing it.
            tableInitialised = 0;
            nodeTable = new List<PathNode>(TABLE_SIZE);
            for (int i = 0; i < TABLE_SIZE; i++)
                nodeTable.Add(new PathNode());

            tableInitialised = 1;
        }

    }
    public enum PathError
    {
        PATH_ERROR_NONE = 0,
        PATH_ERROR_OUT_OF_TRIES = 1,
        PATH_ERROR_OPENLIST_EMPTY = 2
    };
}
