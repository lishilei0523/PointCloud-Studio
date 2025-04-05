using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 网格几何信息
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct MeshGeometryInfo
    {
        /// <summary>
        /// 创建网格几何信息构造器
        /// </summary>
        /// <param name="positions">位置集</param>
        /// <param name="triangleIndices">三角索引集</param>
        public MeshGeometryInfo(Point3F[] positions, int[] triangleIndices)
            : this()
        {
            this.Positions = positions;
            this.TriangleIndices = triangleIndices;
        }

        /// <summary>
        /// 位置集
        /// </summary>
        public readonly Point3F[] Positions;

        /// <summary>
        /// 三角索引集
        /// </summary>
        public readonly int[] TriangleIndices;
    }
}
