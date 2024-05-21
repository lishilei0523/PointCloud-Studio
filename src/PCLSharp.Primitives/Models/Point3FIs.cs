using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 强度坐标点集结构体
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Point3FIs
    {
        /// <summary>
        /// 创建强度坐标点集结构体构造器
        /// </summary>
        /// <param name="points">强度坐标点集指针</param>
        /// <param name="length">长度</param>
        public Point3FIs(IntPtr points, int length)
            : this()
        {
            this.Points = points;
            this.Length = length;
        }

        /// <summary>
        /// 强度坐标点集指针
        /// </summary>
        public readonly IntPtr Points;

        /// <summary>
        /// 长度
        /// </summary>
        public readonly int Length;
    }
}
