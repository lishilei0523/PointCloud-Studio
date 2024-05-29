using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 长方体
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct BoxF
    {
        /// <summary>
        /// 创建长方体构造器
        /// </summary>
        /// <param name="minPoint">最小坐标点</param>
        /// <param name="maxPoint">最大坐标点</param>
        public BoxF(Point3F minPoint, Point3F maxPoint)
            : this()
        {
            this.MinPoint = minPoint;
            this.MaxPoint = maxPoint;
        }

        /// <summary>
        /// 最小坐标点
        /// </summary>
        public readonly Point3F MinPoint;

        /// <summary>
        /// 最大坐标点
        /// </summary>
        public readonly Point3F MaxPoint;
    }
}
