using System;

namespace Sample.Presentation.Models
{
    /// <summary>
    /// 3D坐标点数组
    /// </summary>
    public struct PointArray
    {
        /// <summary>
        /// 点集指针
        /// </summary>
        public IntPtr Points;

        /// <summary>
        /// 长度
        /// </summary>
        public int Length;
    }
}
