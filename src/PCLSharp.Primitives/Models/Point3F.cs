using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 坐标点
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Point3F
    {
        /// <summary>
        /// 创建坐标点构造器
        /// </summary>
        /// <param name="x">X坐标</param>
        /// <param name="y">Y坐标</param>
        /// <param name="z">Z坐标</param>
        public Point3F(float x, float y, float z)
            : this()
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        /// <summary>
        /// X坐标
        /// </summary>
        public readonly float X;

        /// <summary>
        /// Y坐标
        /// </summary>
        public readonly float Y;

        /// <summary>
        /// Z坐标
        /// </summary>
        public readonly float Z;
    }
}
