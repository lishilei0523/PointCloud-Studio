using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 强度坐标点
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Point3FI
    {
        /// <summary>
        /// 创建强度坐标点构造器
        /// </summary>
        /// <param name="x">X坐标</param>
        /// <param name="y">Y坐标</param>
        /// <param name="z">Z坐标</param>
        /// <param name="intensity">强度</param>
        public Point3FI(float x, float y, float z, float intensity)
            : this()
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.Intensity = intensity;
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
        /// 强度
        /// </summary>
        public readonly float Intensity;

        /// <summary>
        /// 是否是非数值
        /// </summary>
        public bool IsNaN()
        {
            if (this.X.Equals(float.NaN) || this.Y.Equals(float.NaN) || this.Z.Equals(float.NaN))
            {
                return true;
            }

            return false;
        }
    }
}
