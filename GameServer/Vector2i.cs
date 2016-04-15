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
    struct Vector2i
    {
        public int X;
        public int Y;

      
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

        public static float EuclideanDist(Vector2i from,Vector2i to)
        {
            float squaredDist = (from.X - to.X) * (from.X - to.X) + (from.Y - to.Y) * (from.Y - to.Y);
            return (float)Math.Sqrt(squaredDist);
        }

        public override bool Equals(Object obj)
        {
            if (obj is Vector2i)
            {
                Vector2i vec = (Vector2i)obj;
                return vec.X == X && vec.Y == Y;
            }
            else
                return false;

            
        }

        public override int GetHashCode()
        {
            return X+Y;
        }

        public static bool operator ==(Vector2i first,Vector2i second)
        {
            return first.X == second.X && first.Y == second.Y;
        }

        public static bool operator !=(Vector2i first, Vector2i second)
        {
            return !(first == second);
        }

        public static Vector2i operator +(Vector2i first,Vector2i second)
        {
            return new Vector2i(first.X + second.X, first.Y + second.Y);
        }

        public static Vector2i operator -(Vector2i first, Vector2i second)
        {
            return new Vector2i(first.X - second.X, first.Y - second.Y);
        }

        public static Vector2i operator *(Vector2i first, int num)
        {
            return new Vector2i(first.X * num, first.Y * num);
        }

        public override string ToString()
        {
            return ("<" + X + ";" + Y+">");
        }

        /// <summary>
        /// Get vector with both coordinates clamped to the provided range
        /// </summary>
        public static Vector2i Clamp(Vector2i vec, int min, int max)
        {
            return new Vector2i(ClampVal(vec.X, min, max), ClampVal(vec.Y, min, max));
        }

        //helper function for Vector2i.Clamp
        private static T ClampVal<T>(T val, T min, T max) where T : IComparable<T>
        {
            if (val.CompareTo(min) < 0) return min;
            else if (val.CompareTo(max) > 0) return max;
            else return val;
        }
    }
}
