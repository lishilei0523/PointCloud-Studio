using System;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// RGBA颜色集结构体
    /// </summary>
    public struct Color4Fs
    {
        /// <summary>
        /// RGBA颜色集指针
        /// </summary>
        public IntPtr Colors;

        /// <summary>
        /// 长度
        /// </summary>
        public int Length;
    }
}
