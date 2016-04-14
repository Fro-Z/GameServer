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
using System.Collections;

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
        protected static int MAX_PATHFIND_TRIES = 3000; /// Max ammount of nodes to open when searching path

        public static bool debugMove = true;

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
            for (tries = 0; job.getOpenNodeCount() != 0; tries++) // Go through the open list while it's not empty
            {
                if (debugOutput)
                    Logger.LogCoreInfo("Going through openlist. Tries: " + tries + " | Objects on list: " + job.getOpenNodeCount());


                if (tries == MAX_PATHFIND_TRIES)
                {
                    path.error = PathError.PATH_ERROR_OUT_OF_TRIES;
                    oot++;
                    //CORE_WARNING("PATH_ERROR_OUT_OF_TRIES");
                    List<Vector2i> rawPath = job.reconstructUnfinishedPath();

                    job.cleanPath(ref rawPath);
                    path.waypoints = job.pathToPosition(ref rawPath); 

                    job.cleanLists();
                    return path;
                }
                else if (job.traverseOpenList(tries == 0))
                {
                    path.error = PathError.PATH_ERROR_NONE;
                    successes++;
                    //CORE_INFO("We finished a path.");

                    List<Vector2i> rawPath = job.reconstructPath();

                    job.cleanPath(ref rawPath);
                    path.waypoints = job.pathToPosition(ref rawPath);
               
                    job.cleanLists();
                    return path;
                }
            }

            if (debugOutput)
                Logger.LogCoreInfo("Going through openlist. Tries: " + tries + " | Objects on list: " + job.getOpenNodeCount());

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
                Vector2i transformed = job.fromPositionToGrid(point);
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
        private List<PathNode> openList, closedList;
        private BitArray openListMask;
        private BitArray closedListMask;

        public Grid[,] map = new Grid[GRID_WIDTH, GRID_HEIGHT];
        public Vector2i start, destination;

        public PathJob()
        {
            openList = new List<PathNode>();
            closedList = new List<PathNode>();
            openListMask = new BitArray(GRID_WIDTH * GRID_HEIGHT);
            closedListMask = new BitArray(GRID_WIDTH * GRID_HEIGHT);

            start = new Vector2i();
            destination = new Vector2i();
            for (var i = 0; i < GRID_WIDTH; i++)
                for (var j = 0; j < GRID_HEIGHT; j++)
                    map[i, j] = new Grid();

        }

        public int getOpenNodeCount()
        {
            return openList.Count;
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

        public Vector2i fromPositionToGrid(Vector2 position)
        {
            AIMesh mesh = Pathfinder.getMesh();
            if (mesh == null)
            {
                Logger.LogCoreError("Tried to get a position without an initialised AIMesh!");
                return new Vector2i();
            }

            return new Vector2i(position / (float)PATH_DEFAULT_BOX_SIZE(mesh.getSize()));
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
            } while (last != null);

            ret.Reverse();

            if(Pathfinder.debugMove)
              DebugHelper.getInstance().ImageFromPath(ref openList, ref closedList,ref ret,ref map,GRID_WIDTH,GRID_HEIGHT);
           
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
        public void cleanPath(ref List<Vector2i> path)
        {
         
            if (path.Count() < 2) return;

            int startSize = path.Count();
            //CORE_WARNING("Cleaning path.. Current size is %d", startSize);

            int lastDirX = 0, lastDirY = 0;
            var prevPoint = path[0];

            List<Vector2i> cleanedPath = new List<Vector2i>();
            cleanedPath.Add(prevPoint);

            for (int i = 1; i<path.Count() ; i++)
            {
                Vector2i currentPoint = path[i];

                int dirX = currentPoint.X - prevPoint.X;
                int dirY = currentPoint.Y - prevPoint.Y;

                //Is waypoint in the same direction?
                if ((dirX==lastDirX) && (dirY==lastDirY))
                {
                    //waypoint in the same direction, last waypoint was not needed
                    cleanedPath.RemoveAt(cleanedPath.Count - 1);
                    cleanedPath.Add(currentPoint);
                }
                else
                {
                    //Update direction change
                    lastDirX = dirX;
                    lastDirY = dirY;
                    //Keep the waypoint

                    cleanedPath.Add(currentPoint);
                }

                prevPoint = currentPoint;
            }
 
            path = cleanedPath;
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
                                if (!isGridNodeOccupied(currentNode.x + dx, currentNode.y + dy) && !isNodeClosed(currentNode.x + dx, currentNode.y + dy)) // Is something here?
                                {
                                    PathNode conflictingNode = isNodeOpen(currentNode.x + dx, currentNode.y + dy); // Nothing is here, did we already add this to the open list?
                                    if (conflictingNode == null) // We did not, add it
                                    {
                                        addToOpenList(new Vector2i(currentNode.x + dx, currentNode.y + dy), currentNode);
                                    }
                                    else if (conflictingNode.g > calcNodeDist(conflictingNode.position, currentNode.position, currentNode.g)) // I found a shorter route to this node.
                                    {
                                        conflictingNode.setParent(currentNode); // Give it a new parent
                                        float nodeDist = calcNodeDist(conflictingNode.position, currentNode.position, currentNode.g);
                                        conflictingNode.setScore(CALC_H(conflictingNode.x, conflictingNode.y, destination.X, destination.Y), nodeDist); // set the new score.
                                    }
                                }
                            }
                        }
                    }
                }
            }


           
            closeNode(currentNode);
            return atDestination;
        }

        /// <summary>
        /// Heuristic function for pathfinding
        /// Calculates distance between two points
        /// </summary>
        /// <returns>Squared distance</returns>
        public float CALC_H(float CURX, float CURY, float ENDX, float ENDY)
        {
            float distX = Math.Abs(CURX - ENDX);
            float distY = Math.Abs(CURY - ENDY);
            return (float)Math.Pow(Math.Sqrt(distX*distX + distY*distY),1.5f);
        }
        /// <summary>
        /// Calculates grid distance from origin node
        /// (Parent's distance + 1) or +sqrt(2) if diagonal
        /// </summary>
        /// <param name="PARENT_G">grid distance of parent node</param>
        /// <returns></returns>
        public float calcNodeDist(Vector2i nodePos, Vector2i parentPos,float parentDist)
        {
            if(nodePos.X==parentPos.X ||nodePos.Y==parentPos.Y)
            {
                return parentDist + 1;
            }
            else
            { //movement was diagonal
                return parentDist + (float)Math.Sqrt(2);
            }
            
        }

        public void addRealPosToOpenList(Vector2 position, PathNode parent)
        {
            addGridPosToOpenList(fromPositionToGrid(position), parent);
        }

        public void addGridPosToOpenList(Vector2i position, PathNode parent)
        {
            float nodeDist = 0;

            if(parent!=null)
                nodeDist = calcNodeDist(position,parent.position,parent.g);

            PathNode node = new PathNode(position, nodeDist, (int)CALC_H(position.X, position.Y, destination.X, destination.Y), parent);
            int nodeID = node.x + node.y * GRID_WIDTH;

            if(nodeID<0 || nodeID>=openListMask.Count)
            {
                Logger.LogCoreError("Pathfinder: opening node with invalid nodeID. x:" + node.x + " y:" + node.y);
                return;
            }
            openListMask.Set(nodeID, true);
            openList.Add(node);
        }

        public void addToOpenList(Vector2i position, PathNode parent)
        {
            addGridPosToOpenList(position, parent);
        }

        public bool isGridNodeOccupied(Vector2 pos)
        {
            return isGridNodeOccupied((int)pos.X, (int)pos.Y);
        }

        public bool isGridNodeOccupied(Vector2i pos)
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
        /// <summary>
        /// Is node with specified coordinates open?
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        public PathNode isNodeOpen(int x, int y)
        {
            int nodeID = x + y * GRID_WIDTH;
            if(openListMask.Get(nodeID))
            {
                //TODO: further optimize lookoop. (Maybe hashtable?)
                foreach (var i in openList)
                {
                    if (i.x == x && i.y == y)
                        return i;
                }
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

                    Vector2i gridPos = fromPositionToGrid(i.Value.getPosition()); // get the position in grid size

                    int radius = ((int)Math.Ceiling((float)i.Value.getCollisionRadius() / (float)PATH_DEFAULT_BOX_SIZE(mesh.getSize()))) / 2; // How many boxes does the radius of this object cover?

                    for (int dx = -radius; dx < radius; dx++) // For the whole radius in the width
                        if (gridPos.X + dx >= 0 && gridPos.X + dx < GRID_WIDTH) // As long as we're in the map (x)
                            for (int dy = -radius; dy < radius; dy++) // for the whole radius in the y
                                if (gridPos.Y + dy >= 0 && gridPos.Y + dy < GRID_HEIGHT) // As long as we're in the map (y)
                                    map[(int)gridPos.X + dx, (int)gridPos.Y + dy].occupied = true; // Occupy this piece of the map.
                }
            }

        
        }
        private static float PATH_DEFAULT_BOX_SIZE(float map_size)
        {
            return map_size / (float)GRID_SIZE;
        }

        public List<Vector2> pathToPosition(ref List<Vector2i> rawPath)
        {
            List<Vector2> ret = new List<Vector2>();
            foreach (Vector2i gridPos in rawPath)
            {
                ret.Add(fromGridToPosition(new Vector2(gridPos.X, gridPos.Y)));
            }
            return ret;
        }

        private bool isNodeClosed(int nodeX, int nodeY)
        {
            int nodeID = nodeX + nodeY * GRID_WIDTH;
            return closedListMask.Get(nodeID);
        }

        private void closeNode(PathNode node)
        {
            closedList.Add(node);
            int nodeID = node.x + GRID_WIDTH * node.y;

            if(nodeID<0 ||nodeID>=closedListMask.Count)
            {
                Logger.LogCoreError("Pathfinder: closing node with invalid nodeID. x:" + node.x + " y:" + node.y);
                return;
            }

            closedListMask.Set(nodeID, true);
            openListMask.Set(nodeID, false);
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
        //compatibility methods for old code
        public int x
        {
            get { return position.X; }
            set { position.X = value; }
        }

        public int y
        {
            get { return position.Y; }
            set { position.Y = value; }
        }
        
       

        public Vector2i position;

        public float h; //heuristic distance to target
        public float g; //calculated distance from origin

        public PathNode parent;
        private static int tableInitialised;
        private const int TABLE_SIZE = (2 << 15);
        private static List<PathNode> nodeTable = new List<PathNode>();

        public PathNode()
        {
            InitTable();
        }

        public PathNode(int x, int y, float ag, float ah, PathNode p)
        {
            Init(new Vector2i(x,y), ag, ah, p);
        }

        public PathNode(Vector2i pos, float ag, float ah, PathNode p)
        {
            Init(pos, ag, ah, p);
        }

        public void Init(Vector2i pos, float ag, float ah, PathNode p)
        {
            InitTable();
            position = pos;
            h = ah;
            g = ag;
            parent = p;
        }

        public void setScore(float ah, float ag)
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
