using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// RGBA颜色
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Color4F
    {
        /// <summary>
        /// 创建RGBA颜色构造器
        /// </summary>
        /// <param name="r">R值</param>
        /// <param name="g">G值</param>
        /// <param name="b">B值</param>
        /// <param name="a">A值</param>
        public Color4F(float r, float g, float b, float a)
            : this()
        {
            this.A = a;
            this.R = r;
            this.G = g;
            this.B = b;
        }

        /// <summary>
        /// R值
        /// </summary>
        public float R;

        /// <summary>
        /// G值
        /// </summary>
        public float G;

        /// <summary>
        /// B值
        /// </summary>
        public float B;

        /// <summary>
        /// A值
        /// </summary>
        public float A;
    }
}
