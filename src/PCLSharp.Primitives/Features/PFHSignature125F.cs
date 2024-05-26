using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Features
{
    /// <summary>
    /// PFH描述子
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct PFHSignature125F
    {
        /// <summary>
        /// 创建PFH描述子构造器
        /// </summary>
        /// <param name="features">特征向量</param>
        public PFHSignature125F(float[] features)
            : this()
        {
            this.Features = features;
        }

        /// <summary>
        /// 特征向量
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 125)]
        public readonly float[] Features;
    }
}
