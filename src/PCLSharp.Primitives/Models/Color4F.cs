using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// RGBA颜色
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Color4F
    {
        /// <summary>
        /// 创建RGBA颜色构造器
        /// </summary>
        /// <param name="r">R值</param>
        /// <param name="g">G值</param>
        /// <param name="b">B值</param>
        /// <param name="a">A值</param>
        public Color4F(byte r, byte g, byte b, byte a)
            : this()
        {
            this.R = r;
            this.G = g;
            this.B = b;
            this.A = a == 0 ? byte.MaxValue : a;
        }

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
