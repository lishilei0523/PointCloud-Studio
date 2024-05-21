using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// RGB颜色集结构体
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Color3Fs
    {
        /// <summary>
        /// 创建RGB颜色集结构体构造器
        /// </summary>
        /// <param name="colors">RGB颜色集指针</param>
        /// <param name="length">长度</param>
        public Color3Fs(IntPtr colors, int length)
            : this()
        {
            this.Colors = colors;
            this.Length = length;
        }

        /// <summary>
        /// RGB颜色集指针
        /// </summary>
        public readonly IntPtr Colors;

        /// <summary>
        /// 长度
        /// </summary>
        public readonly int Length;
    }
}
