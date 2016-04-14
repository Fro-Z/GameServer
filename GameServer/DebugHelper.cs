using LeagueSandbox.GameServer.Core.Logic;
using LeagueSandbox.GameServer.Core.Logic.PacketHandlers;
using LeagueSandbox.GameServer.Core.Logic.PacketHandlers.Packets;
using LeagueSandbox.GameServer.Logic.GameObjects;
using LeagueSandbox.GameServer.Logic.Packets;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using static ENet.Native;
using System.Drawing;
using LeagueSandbox.GameServer;

namespace LeagueSandbox
{
    /// <summary>
    /// Debug helper class. Provides range of function to help debugging
    /// </summary>
    unsafe class DebugHelper
    {
       

        public static DebugHelper getInstance()
        {
            if (_instance == null)
                _instance = new DebugHelper();

            return _instance;
        }

        /// <summary>
        /// Ping at position
        /// WARNING: you must set parameters 'game' and 'peer' before calling this function
        /// </summary>
        /// <param name="pos"></param>
        /// <param name="pingType"></param>
        public void PingAtLocation(Vector2 pos, Pings pingType = Pings.Ping_Danger)
        {
            if (game == null)
                return;

            if (peer == null)
                return;

            
            var response = new AttentionPingAns(game.getPeerInfo(peer), new AttentionPing { x = pos.X, y = pos.Y, targetNetId = 0, type = Pings.Ping_Danger });
            PacketHandlerManager.getInstace().broadcastPacketTeam(game.getPeerInfo(peer).getTeam(), response, Channel.CHL_S2C);
        }

           public void ImageFromPath(ref List<PathNode> openList,ref List<PathNode> closedList,ref List<Vector2i> pathPoints,ref Grid[,] map,int GRID_WIDTH,int GRID_HEIGHT)
        {
            Bitmap bitmap = new Bitmap(GRID_WIDTH, GRID_HEIGHT);
            

        //DEBUG
            List<Vector3> imageData = new List<Vector3>();
           for (int i = 0; i < GRID_HEIGHT; i++) //dump basic image data
               for (int j = 0; j < GRID_WIDTH; j++)
               {
                   if (map[j, i].occupied)
                       imageData.Add(new Vector3(255, 255, 255));
                   else
                       imageData.Add(new Vector3(0, 0, 0));

                    Color color = Color.Black;
                    if (map[j, i].occupied)
                        color = Color.White;

                    bitmap.SetPixel(j, GRID_HEIGHT-i-1, color);
               }
           foreach(PathNode node in closedList)
           {
               bitmap.SetPixel(node.x, GRID_HEIGHT-1 - node.y,Color.Red); //Display closed nodes red
           }
           foreach (PathNode node in openList)
           {
                bitmap.SetPixel(node.x, GRID_HEIGHT - 1 - node.y, Color.Green); //Display open nodes green
            }
           foreach (Vector2i node in pathPoints)
           {
                bitmap.SetPixel(node.x, GRID_HEIGHT - 1 - node.y, Color.Blue); //Display path nodes blue
            }

           try
           {
                bitmap.Save("C:/mapImage.png");
            }
           catch (System.Exception ex)
           {
                Logger.LogCoreError("Could not save debug bitmap");
           }
            
           
        }



        public Game game;
       public  ENetPeer* peer;
        private static DebugHelper _instance;

    }

}
