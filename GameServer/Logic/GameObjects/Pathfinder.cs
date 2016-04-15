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
using Priority_Queue;
using System.Diagnostics;

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
        protected static int MAX_PATHFIND_TRIES = 1000; /// Max ammount of nodes to open when searching path

        public static bool debugMove = true;
        public static bool ignoreUnits = true;

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
                else if (job.traverseOpenListJMP())
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

            Stopwatch sw = new Stopwatch();
            sw.Start();
            Path p = getPath(from, to, PATH_DEFAULT_BOX_SIZE(getMesh().getSize()));
            sw.Stop();
            Logger.LogCoreInfo("Pathfinding took " + sw.ElapsedMilliseconds + "ms");

            return p;
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

        public List<Vector2> getWaypoints()
        {
            return waypoints;
        }
    };

    class PathJob
    {
        private static int GRID_SIZE = 512;
        private static int GRID_WIDTH = GRID_SIZE;
        private static int GRID_HEIGHT = GRID_SIZE;
        private SimplePriorityQueue<PathNode> openList;
        private List<PathNode> closedList;
        private BitArray openListMask;
        private BitArray closedListMask;


        public Grid[,] map;

        public Vector2i start, destination;

        public PathJob()
        {
            openList = new SimplePriorityQueue<PathNode>();
            closedList = new List<PathNode>();
            openListMask = new BitArray(GRID_WIDTH * GRID_HEIGHT);
            closedListMask = new BitArray(GRID_WIDTH * GRID_HEIGHT);


            map = new Grid[GRID_WIDTH, GRID_HEIGHT];

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
                 if (node.getScore() < lowestScore)
                 {
                     lowestScore = node.getScore();
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

        /// <summary>
        /// Traverses graph using Jump Point Search algorithm
        /// </summary>
        /// <returns>true if opened node was the destination node</returns>
        public bool traverseOpenListJMP()
        {
            if (openList.Count == 0)
                return false;

            PathNode currentNode = openList.First;

            openList.Remove(currentNode);
                       

            bool atDestination = false;

            if (currentNode.position == destination)
                atDestination = true;

            Vector2i parentPos = currentNode.position;
            if (currentNode.parent != null)
                parentPos = currentNode.parent.position;

            if(!atDestination)
            {
                List<Vector2i> neightbours = getNeighbours(currentNode.position, parentPos);
                foreach (Vector2i neighbour in neightbours)
                {
                    if (isNodeOpen(neighbour) || isNodeClosed(neighbour))
                        continue;

                    Vector2i direction = neighbour - currentNode.position;

                    if (jump(currentNode, direction))
                        break;

                }
            }
         

            closeNode(currentNode);
            return atDestination;
        }
      
        /// <summary>
        /// Jumps from node in direction
        /// </summary>
        /// <returns>true if new node is destination node</returns>
        private bool jump(PathNode from,Vector2i direction)
        {
            direction = Vector2i.Clamp(direction, -1, 1);

            Vector2i curPos = from.position;

           

            do
            {
                curPos += direction;

                if (isGridNodeOccupied(curPos))
                    return false;

                if (curPos == destination)
                {
                    addToOpenList(curPos, from);
                    return true;
                }

                //Diagonal case
                if (direction.X != 0 && direction.Y != 0)
                {
                    if (diagonalNeighbourCheck(curPos, direction))
                    {
                        if(!isNodeClosed(curPos) && !isNodeOpen(curPos))
                            addToOpenList(curPos, from);
                        return false;
                    }

                    Vector2i horizontalDir = direction;
                    horizontalDir.Y = 0;
                    Vector2i verticalDir = direction;
                    verticalDir.X = 0;

                    // Check in horizontal and vertical directions for forced neighbors
                    // This is a special case for diagonal direction
                    if (testJump(curPos, horizontalDir) || testJump(curPos, verticalDir))
                    {
                        if (!isNodeClosed(curPos) && !isNodeOpen(curPos))
                            addToOpenList(curPos, from);
                        return false;
                    }

                }
                else //Non-diagonal case
                {
                    if (nonDiagonalNeighbourCheck(curPos, direction))
                    {
                        if (!isNodeClosed(curPos) && !isNodeOpen(curPos))
                            addToOpenList(curPos, from);
                        return false;
                    }
                }

            //Continue until we find either wall or a forced neighbour   
            }
            while (true);

            return false;
        }


        /// <summary>
        /// Non-diagonal only jump function, that does not create new nodes
        /// </summary>
        /// <param name="currentPos"></param>
        /// <param name="direction"></param>
        /// <returns>true if found a forced neighbour</returns>
        private bool testJump(Vector2i currentPos,Vector2i direction)
        {
            if(direction.X!=0 && direction.Y!=0)
            {
                Logger.LogCoreError("PathfindingError: Non diagonal fuction called with diagonal direction!");
                return false;
            }

            Vector2i nextPos = currentPos;

            do
            {
                nextPos += direction;

                if (isGridNodeOccupied(nextPos))
                    return false;

                if (nextPos == destination)
                    return true;

            }
            while (!nonDiagonalNeighbourCheck(nextPos, direction)); //run until we either find forced neighbour or a wall

            return true;
        }



        /// <summary>
        /// Heuristic function for pathfinding
        /// Using euclidean distance between two points
        /// </summary>
        /// <returns>Squared distance</returns>
        public float calcNodeH(Vector2i from,Vector2i to)
        {
            return Vector2i.EuclideanDist(from, to);
        }


       
        /// <summary>
        /// Calculates real distance from origin node (only for straight lines and diagonals)
        /// </summary>
        /// <param name="PARENT_G">grid distance of parent node</param>
        /// <returns></returns>
        public float calcNodeG(Vector2i nodePos, Vector2i parentPos,float parentDist)
        {
            return Vector2i.EuclideanDist(parentPos, nodePos)+parentDist;   
        }

        public void addRealPosToOpenList(Vector2 position, PathNode parent)
        {
            addGridPosToOpenList(fromPositionToGrid(position), parent);
        }

        public void addGridPosToOpenList(Vector2i position, PathNode parent)
        {
            float nodeDist = 0;

            if(parent!=null)
                nodeDist = calcNodeG(position,parent.position,parent.g);

            PathNode node = new PathNode(position, nodeDist, calcNodeH(position, destination), parent);
            int nodeIndex = node.x + node.y * GRID_WIDTH;

            if(nodeIndex<0 || nodeIndex>=openListMask.Count)
            {
                Logger.LogCoreError("Pathfinder: opening node with invalid nodeID. x:" + node.x + " y:" + node.y);
                return;
            }

            if(openListMask.Get(nodeIndex))
            {
                Logger.LogCoreError("opening node that is already open!");
                return;
            }

            if (closedListMask.Get(nodeIndex))
            {
                Logger.LogCoreError("opening node that is already closed!");
                return;
            }

            openListMask.Set(nodeIndex, true);
            openList.Enqueue(node, node.getScore());
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
            return isGridNodeOccupied(pos.X, pos.Y);
        }

        public bool isGridNodeOccupied(int x, int y)
        {
            if ((x >= 0 && x < GRID_SIZE) && (y >= 0 && y < GRID_SIZE))
            {
                return map[x, y].isOccupied();
            }
            else return true;
        }

        public bool isNodeOpen(Vector2i pos)
        {
            return isNodeOpen(pos.X, pos.Y);
        }
        public bool isNodeOpen(int nodeX, int nodeY)
        {
            int nodeIndex = nodeX + nodeY * GRID_WIDTH;

            if (nodeIndex < 0)
                return false;

            return openListMask.Get(nodeIndex);
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

            if (chart != null && !Pathfinder.ignoreUnits)
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

        private bool isNodeClosed(Vector2i pos)
        {
            return isNodeClosed(pos.X, pos.Y);
        }

        private bool isNodeClosed(int nodeX, int nodeY)
        {
            int nodeIndex = nodeX + nodeY * GRID_WIDTH;

            if (nodeIndex < 0)
                return false;

            return closedListMask.Get(nodeIndex);
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


        
        /// <summary>
        /// JMP Search:
        /// Get all neighbours for position that could potentially have shorter path
        /// through 'from' -> 'position' route
        /// </summary>
        /// <param name="position"></param>
        /// <param name="from"></param>
        /// <returns></returns>
        private List<Vector2i> getNeighbours(Vector2i position,Vector2i from)
        {
            List<Vector2i> neighbours = new List<Vector2i>();
            Vector2i dir = Vector2i.Clamp(position - from, -1, 1);
           

            if(position==from) //first node, add all possible neighbours
                for (int i = -1; i <= 1; i++)
                    for (int j = -1; j <= 1; j++)
                    {
                        if (!(j == 0 && i == 0))
                        {
                            Vector2i newPos = new Vector2i(i, j) + position;
                                neighbours.Add(newPos);
                        }
                            
                    }

            else
            {
                if(dir.Y==0) //horizontal movement
                {
                    Vector2i testPos1 = position + new Vector2i(0, 1);
                    Vector2i testPos2 = position + new Vector2i(0, -1);

                    //Add a node if a way to it is occupied
                    if (isGridNodeOccupied(testPos1))
                            neighbours.Add(testPos1 + dir);

                    if (isGridNodeOccupied(testPos2))
                            neighbours.Add(testPos2 + dir);

                    neighbours.Add(position + dir);
                }
                else
                if(dir.X==0) //vertical movement
                {
                    Vector2i testPos1 = position + new Vector2i(1, 0);
                    Vector2i testPos2 = position + new Vector2i(-1, 0);

                    //Add a node if a way to it is occupied
                    if (isGridNodeOccupied(testPos1))
                        neighbours.Add(testPos1 + dir);

                    if (isGridNodeOccupied(testPos2))
                        neighbours.Add(testPos2 + dir);

                    neighbours.Add(position + dir);
                }
                else //diagonal movement
                {
                    Vector2i horizontalDir = new Vector2i(dir.X, 0);
                    Vector2i testPos1 = position - horizontalDir;
                    Vector2i verticalDir = new Vector2i(0, dir.Y);
                    Vector2i testPos2 = position - verticalDir;

                  //  if (isGridNodeOccupied(testPos1))
                   // {
                        neighbours.Add(testPos1 + dir - horizontalDir);
                        neighbours.Add(testPos1 + dir);
                   // }
                        

                  //  if(isGridNodeOccupied(testPos2))
                  //  {
                        neighbours.Add(testPos2 + dir - verticalDir);
                        neighbours.Add(testPos2 + dir);
                   // }
                        

                    neighbours.Add(position + dir);
                }

            }
            return neighbours;
        }


        /// <summary>
        /// JMP Search: Tests if current position has a forced neighbour
        /// </summary>
        /// <param name="pos">node position</param>
        /// <param name="dir">direction of arrival</param>
        private bool nonDiagonalNeighbourCheck(Vector2i pos, Vector2i dir)
        {
            if(Pathfinder.debugMove)
            {
                if (dir.X != 0 && dir.Y != 0)
                    Logger.LogCoreError("nonDiagonalNeighbourCheck ran with diagonal direction!");
            }

            if(dir.X==0) //vertical direction, test for horizontal occupation
            {
                Vector2i testPoint1 = pos + new Vector2i(1, 0);
                Vector2i testPoint2 = pos + new Vector2i(-1, 0);

                //neighbour is forced if testPoint blocks access to unoccupied node
                if (isGridNodeOccupied(testPoint1) && !isGridNodeOccupied(testPoint1 + dir))
                    return true;

                if (isGridNodeOccupied(testPoint2) && !isGridNodeOccupied(testPoint2 + dir))
                    return true;
            }
            else //assume horizontal direction, test for vertical occupation
            {
                Vector2i testPoint1 = pos + new Vector2i(0, 1);
                Vector2i testPoint2 = pos + new Vector2i(0, -1);

                if (isGridNodeOccupied(testPoint1) && !isGridNodeOccupied(testPoint1 + dir))
                    return true;

                if (isGridNodeOccupied(testPoint2) && !isGridNodeOccupied(testPoint2 + dir))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// JMP Search: Tests if current position has a forced neighbour
        /// </summary>
        /// <param name="pos">node position</param>
        /// <param name="dir">direction of arrival</param>
        private bool diagonalNeighbourCheck(Vector2i pos, Vector2i dir)
        {
            Vector2i horizontalDir = new Vector2i(dir.X, 0);
            Vector2i testPos1 = pos - horizontalDir;
            Vector2i verticalDir = new Vector2i(0, dir.Y);
            Vector2i testPos2 = pos - verticalDir;

            if (isGridNodeOccupied(testPos1))
            {
                if (!isGridNodeOccupied(testPos1 + dir - horizontalDir))
                    return true;

                if (!isGridNodeOccupied(testPos1 + dir))
                    return true;
            }
               

            if (isGridNodeOccupied(testPos2))
            {
                if (!isGridNodeOccupied(testPos2 + dir - verticalDir))
                    return true;

                if (!isGridNodeOccupied(testPos2 + dir))
                    return true;
            }
                

            return false;
        }

        internal void PrintImage()
        {
            DebugHelper.getInstance().ImageFromPath(openList, closedList, new List<Vector2i>(),map, GRID_WIDTH, GRID_HEIGHT);
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

        public float getScore()
        {
            return g + h;
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
