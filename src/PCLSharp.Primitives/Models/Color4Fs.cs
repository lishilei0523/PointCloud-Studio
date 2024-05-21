using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// RGBA颜色集结构体
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Color4Fs
    {
        /// <summary>
        /// 创建RGBA颜色集结构体构造器
        /// </summary>
        /// <param name="colors">RGBA颜色集指针</param>
        /// <param name="length">长度</param>
        public Color4Fs(IntPtr colors, int length)
            : this()
        {
            this.Colors = colors;
            this.Length = length;
        }

        /// <summary>
        /// RGBA颜色集指针
        /// </summary>
        public readonly IntPtr Colors;

        /// <summary>
        /// 长度
        /// </summary>
        public readonly int Length;
    }
}
