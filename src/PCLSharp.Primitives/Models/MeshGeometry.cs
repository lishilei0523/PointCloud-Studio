using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 网格几何
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct MeshGeometry
    {
        /// <summary>
        /// 创建网格几何构造器
        /// </summary>
        /// <param name="positions">位置集指针</param>
        /// <param name="positionsLength">位置集长度</param>
        /// <param name="triangleIndices">三角索引集指针</param>
        /// <param name="triangleIndicesLength">三角索引集长度</param>
        public MeshGeometry(IntPtr positions, int positionsLength, IntPtr triangleIndices, int triangleIndicesLength)
            : this()
        {
            this.Positions = positions;
            this.PositionsLength = positionsLength;
            this.TriangleIndices = triangleIndices;
            this.TriangleIndicesLength = triangleIndicesLength;
        }

        /// <summary>
        /// 位置集指针
        /// </summary>
        public readonly IntPtr Positions;

        /// <summary>
        /// 位置集长度
        /// </summary>
        public readonly int PositionsLength;

        /// <summary>
        /// 三角索引集指针
        /// </summary>
        public readonly IntPtr TriangleIndices;

        /// <summary>
        /// 三角索引集长度
        /// </summary>
        public readonly int TriangleIndicesLength;
    }
}
