﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ENet;
using static ENet.Native;
using LeagueSandbox.GameServer.Logic.Packets;

namespace LeagueSandbox.GameServer.Core.Logic.PacketHandlers.Packets
{
    class HandleLoadPing : IPacketHandler
    {
        public unsafe bool HandlePacket(ENetPeer* peer, byte[] data, Game game)
        {
            var loadInfo = new PingLoadInfo(data);
            var peerInfo = game.getPeerInfo(peer);
            if (peerInfo == null)
                return false;
            var response = new PingLoadInfo(loadInfo, peerInfo.userId);

            //Logging->writeLine("loaded: %f, ping: %f, %f", loadInfo->loaded, loadInfo->ping, loadInfo->f3);
            return PacketHandlerManager.getInstace().broadcastPacket(response, Channel.CHL_LOW_PRIORITY, PacketFlags.None);
        }
    }
}
