using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace LeagueSandbox.GameServer
{
    /// <summary>
    /// Vector of 2 integers
    /// </summary>
    class Vector2i
    {
        public int X;
        public int Y;

        public Vector2i()
        {
            X = 0;
            Y = 0;
        }

        public Vector2i(Vector2 floatVec)
        {
            this.X = (int)floatVec.X;
            this.Y = (int)floatVec.Y;
        }
        public Vector2i(int x, int y)
        {
            this.X = x;
            this.Y = y;
        }
    }
}
