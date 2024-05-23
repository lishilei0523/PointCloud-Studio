using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 坐标点法向量集结构体
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Point3Normal3s
    {
        /// <summary>
        /// 创建坐标点法向量集结构体构造器
        /// </summary>
        /// <param name="pointNormals">坐标点法向量集指针</param>
        /// <param name="length">长度</param>
        public Point3Normal3s(IntPtr pointNormals, int length)
            : this()
        {
            this.PointNormals = pointNormals;
            this.Length = length;
        }

        /// <summary>
        /// 坐标点法向量集指针
        /// </summary>
        public readonly IntPtr PointNormals;

        /// <summary>
        /// 长度
        /// </summary>
        public readonly int Length;
    }
}
