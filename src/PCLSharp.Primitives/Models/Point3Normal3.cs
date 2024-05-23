using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 坐标点法向量
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Point3Normal3
    {
        /// <summary>
        /// 创建坐标点法向量构造器
        /// </summary>
        /// <param name="x">X坐标</param>
        /// <param name="y">Y坐标</param>
        /// <param name="z">Z坐标</param>
        /// <param name="nx">法向量X坐标</param>
        /// <param name="ny">法向量Y坐标</param>
        /// <param name="nz">法向量Z坐标</param>
        public Point3Normal3(float x, float y, float z, float nx, float ny, float nz)
            : this()
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.NX = nx;
            this.NY = ny;
            this.NZ = nz;
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
        /// 法向量X坐标
        /// </summary>
        public readonly float NX;

        /// <summary>
        /// 法向量Y坐标
        /// </summary>
        public readonly float NY;

        /// <summary>
        /// 法向量Z坐标
        /// </summary>
        public readonly float NZ;
    }
}
