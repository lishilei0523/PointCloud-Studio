using System;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 坐标点集结构体
    /// </summary>
    public struct Point3Fs
    {
        /// <summary>
        /// 坐标集指针
        /// </summary>
        public IntPtr Points;

        /// <summary>
        /// 长度
        /// </summary>
        public int Length;
    }
}
