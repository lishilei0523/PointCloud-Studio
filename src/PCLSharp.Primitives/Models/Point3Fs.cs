using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 坐标点集结构体
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Point3Fs
    {
        /// <summary>
        /// 创建坐标点集结构体
        /// </summary>
        /// <param name="points">坐标集指针</param>
        /// <param name="length">长度</param>
        public Point3Fs(IntPtr points, int length)
            : this()
        {
            this.Points = points;
            this.Length = length;
        }

        /// <summary>
        /// 坐标集指针
        /// </summary>
        public readonly IntPtr Points;

        /// <summary>
        /// 长度
        /// </summary>
        public readonly int Length;
    }
}
