using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 坐标点颜色
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Point3Color4
    {
        /// <summary>
        /// 创建坐标点颜色构造器
        /// </summary>
        /// <param name="x">X坐标</param>
        /// <param name="y">Y坐标</param>
        /// <param name="z">Z坐标</param>
        /// <param name="r">R值</param>
        /// <param name="g">G值</param>
        /// <param name="b">B值</param>
        /// <param name="a">A值</param>
        public Point3Color4(float x, float y, float z, byte r, byte g, byte b, byte a)
            : this()
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.A = a;
            this.R = r;
            this.G = g;
            this.B = b;
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

        /// <summary>
        /// R值
        /// </summary>
        public readonly byte R;

        /// <summary>
        /// G值
        /// </summary>
        public readonly byte G;

        /// <summary>
        /// B值
        /// </summary>
        public readonly byte B;

        /// <summary>
        /// A值
        /// </summary>
        public readonly byte A;
    }
}
