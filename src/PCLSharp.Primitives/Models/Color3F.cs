using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// RGB颜色
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct Color3F
    {
        /// <summary>
        /// 创建RGB颜色构造器
        /// </summary>
        /// <param name="r">R值</param>
        /// <param name="g">G值</param>
        /// <param name="b">B值</param>
        public Color3F(float r, float g, float b)
            : this()
        {
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
    }
}
