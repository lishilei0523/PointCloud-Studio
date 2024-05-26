using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 坐标点颜色集
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Point3Color4s
    {
        /// <summary>
        /// 创建坐标点颜色集构造器
        /// </summary>
        /// <param name="pointColors">坐标点颜色集指针指针</param>
        /// <param name="length">长度</param>
        public Point3Color4s(IntPtr pointColors, int length)
            : this()
        {
            this.PointColors = pointColors;
            this.Length = length;
        }

        /// <summary>
        /// 坐标点颜色集指针
        /// </summary>
        public readonly IntPtr PointColors;

        /// <summary>
        /// 长度
        /// </summary>
        public readonly int Length;
    }
}
