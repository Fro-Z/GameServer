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
        public int x;
        public int y;

        public Vector2i()
        {
            x = 0;
            y = 0;
        }

        public Vector2i(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
    }
}
