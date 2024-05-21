using System;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 强度坐标点集结构体
    /// </summary>
    public struct Point3FIs
    {
        /// <summary>
        /// 强度坐标点集指针
        /// </summary>
        public IntPtr Points;

        /// <summary>
        /// 长度
        /// </summary>
        public int Length;
    }
}
