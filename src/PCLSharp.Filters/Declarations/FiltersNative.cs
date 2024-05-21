using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Filters.Declarations
{
    /// <summary>
    /// 点云滤波声明
    /// </summary>
    public static class FiltersNative
    {
        #region # 适用直通滤波 —— static extern IntPtr ApplyPassThrogh(Point3F[] points...
        /// <summary>
        /// 适用直通滤波
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="axis">过滤坐标轴</param>
        /// <param name="limitMin">过滤范围最小值</param>
        /// <param name="limixMax">过滤范围最大值</param>
        /// <returns>过滤后点集</returns>
        [DllImport(AssemblyNames.Filters, EntryPoint = "applyPassThrogh")]
        public static extern IntPtr ApplyPassThrogh(Point3F[] points, int length, string axis, float limitMin, float limixMax);
        #endregion

        #region # 适用均匀采样 —— static extern IntPtr ApplyUniformSampling(Point3F[] points...
        /// <summary>
        /// 适用均匀采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="radius">采样半径</param>
        /// <returns>过滤后点集</returns>
        [DllImport(AssemblyNames.Filters, EntryPoint = "applyUniformSampling")]
        public static extern IntPtr ApplyUniformSampling(Point3F[] points, int length, float radius);
        #endregion

        #region # 适用体素降采样 —— static extern IntPtr ApplyVoxelGrid(Point3F[] points...
        /// <summary>
        /// 适用体素降采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="leafSize">叶尺寸</param>
        /// <returns>过滤后点集</returns>
        [DllImport(AssemblyNames.Filters, EntryPoint = "applyVoxelGrid")]
        public static extern IntPtr ApplyVoxelGrid(Point3F[] points, int length, float leafSize);
        #endregion

        #region # 适用离群点移除 —— static extern IntPtr ApplyOutlierRemoval(Point3F[] points...
        /// <summary>
        /// 适用离群点移除
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="meanK">平均距离估计的最近邻居的数量</param>
        /// <param name="stddevMult">标准差阈值系数</param>
        /// <returns>过滤后点集</returns>
        [DllImport(AssemblyNames.Filters, EntryPoint = "applyOutlierRemoval")]
        public static extern IntPtr ApplyOutlierRemoval(Point3F[] points, int length, int meanK, float stddevMult);
        #endregion

        #region # 释放资源 —— static extern void Dispose(IntPtr pointer)
        /// <summary>
        /// 释放资源
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Filters, EntryPoint = "dispose")]
        public static extern void Dispose(IntPtr pointer);
        #endregion
    }
}
