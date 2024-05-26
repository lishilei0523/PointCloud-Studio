using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Features
{
    /// <summary>
    /// FPFH描述子
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct FPFHSignature33F
    {
        /// <summary>
        /// 创建FPFH描述子构造器
        /// </summary>
        /// <param name="features">特征向量</param>
        public FPFHSignature33F(float[] features)
            : this()
        {
            this.Features = features;
        }

        /// <summary>
        /// 特征向量
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 33)]
        public readonly float[] Features;
    }
}
