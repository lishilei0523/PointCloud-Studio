using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云法向量声明
    /// </summary>
    internal static class NormalsNative
    {
        #region # 估算法向量 —— static extern IntPtr EstimateNormalsByK(Point3F[] points...
        /// <summary>
        /// 估算法向量
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="k">搜索近邻数量</param>
        /// <returns>法向量集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "estimateNormalsByK")]
        public static extern IntPtr EstimateNormalsByK(Point3F[] points, int length, int k);
        #endregion

        #region # 估算法向量 —— static extern IntPtr EstimateNormalsByRadius(Point3F[] points...
        /// <summary>
        /// 估算法向量
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="radius">搜索半径</param>
        /// <returns>法向量集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "estimateNormalsByRadius")]
        public static extern IntPtr EstimateNormalsByRadius(Point3F[] points, int length, float radius);
        #endregion

        #region # 估算法向量 (OMP) —— static extern IntPtr EstimateNormalsByKP(Point3F[] points...
        /// <summary>
        /// 估算法向量 (OMP)
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="k">搜索近邻数量</param>
        /// <returns>法向量集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "estimateNormalsByKP")]
        public static extern IntPtr EstimateNormalsByKP(Point3F[] points, int length, int k);
        #endregion

        #region # 估算法向量 (OMP) —— static extern IntPtr EstimateNormalsByRadiusP(Point3F[] points...
        /// <summary>
        /// 估算法向量 (OMP)
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="radius">搜索半径</param>
        /// <returns>法向量集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "estimateNormalsByRadiusP")]
        public static extern IntPtr EstimateNormalsByRadiusP(Point3F[] points, int length, float radius);
        #endregion
    }
}
