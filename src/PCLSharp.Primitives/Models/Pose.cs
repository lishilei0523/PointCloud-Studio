using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 位姿
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Pose
    {
        /// <summary>
        /// 创建位姿构造器
        /// </summary>
        /// <param name="x">X轴位置</param>
        /// <param name="y">Y轴位置</param>
        /// <param name="z">Z轴位置</param>
        /// <param name="rx">X轴旋转角度</param>
        /// <param name="ry">Y轴旋转角度</param>
        /// <param name="rz">Z轴旋转角度</param>
        public Pose(float x, float y, float z, float rx, float ry, float rz)
            : this()
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.RX = rx;
            this.RY = ry;
            this.RZ = rz;
        }

        /// <summary>
        /// X轴位置
        /// </summary>
        public readonly float X;

        /// <summary>
        /// Y轴位置
        /// </summary>
        public readonly float Y;

        /// <summary>
        /// Z轴位置
        /// </summary>
        public readonly float Z;

        /// <summary>
        /// X轴旋转角度
        /// </summary>
        public readonly float RX;

        /// <summary>
        /// Y轴旋转角度
        /// </summary>
        public readonly float RY;

        /// <summary>
        /// Z轴旋转角度
        /// </summary>
        public readonly float RZ;
    }
}
