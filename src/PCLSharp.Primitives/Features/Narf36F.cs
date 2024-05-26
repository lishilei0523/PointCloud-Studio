using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Features
{
    /// <summary>
    /// NARF描述子
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Narf36F
    {
        /// <summary>
        /// 创建NARF描述子构造器
        /// </summary>
        /// <param name="x">X坐标</param>
        /// <param name="y">Y坐标</param>
        /// <param name="z">Z坐标</param>
        /// <param name="pitch">俯仰角(RX)</param>
        /// <param name="yaw">偏航角(RY)</param>
        /// <param name="roll">翻滚角(RZ)</param>
        /// <param name="features">特征向量</param>
        public Narf36F(float x, float y, float z, float pitch, float yaw, float roll, float[] features)
            : this()
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
            this.Pitch = pitch;
            this.Yaw = yaw;
            this.Roll = roll;
            this.Features = features;
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
        /// 俯仰角(RX)
        /// </summary>
        public readonly float Pitch;

        /// <summary>
        /// 偏航角(RY)
        /// </summary>
        public readonly float Yaw;

        /// <summary>
        /// 翻滚角(RZ)
        /// </summary>
        public readonly float Roll;

        /// <summary>
        /// 特征向量
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 36)]
        public readonly float[] Features;
    }
}
