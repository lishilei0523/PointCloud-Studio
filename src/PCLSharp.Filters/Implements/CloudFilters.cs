using PCLSharp.Filters.Declarations;
using PCLSharp.Filters.Interfaces;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;

namespace PCLSharp.Filters.Implements
{
    /// <summary>
    /// 点云滤波实现
    /// </summary>
    public class CloudFilters : ICloudFilters
    {
        #region # 适用直通滤波 —— Point3F[] ApplyPassThrogh(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用直通滤波
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="axis">过滤坐标轴</param>
        /// <param name="limitMin">过滤范围最小值</param>
        /// <param name="limixMax">过滤范围最大值</param>
        /// <returns>过滤后点集</returns>
        public Point3F[] ApplyPassThrogh(IEnumerable<Point3F> points, string axis, float limitMin, float limixMax)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = FiltersNative.ApplyPassThrogh(points_, points_.Length, axis, limitMin, limixMax);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] filteredPoints = point3Fs.ToPoint3Fs();
            FiltersNative.Dispose(pointer);

            return filteredPoints;
        }
        #endregion

        #region # 适用随机采样 —— Point3F[] ApplyRandomSampling(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用随机采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="seed">随机种子</param>
        /// <param name="samplesCount">采样数量</param>
        /// <returns>过滤后点集</returns>
        public Point3F[] ApplyRandomSampling(IEnumerable<Point3F> points, int seed, int samplesCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = FiltersNative.ApplyRandomSampling(points_, points_.Length, seed, samplesCount);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] filteredPoints = point3Fs.ToPoint3Fs();
            FiltersNative.Dispose(pointer);

            return filteredPoints;
        }
        #endregion

        #region # 适用均匀采样 —— Point3F[] ApplyUniformSampling(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用均匀采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="radius">采样半径</param>
        /// <returns>过滤后点集</returns>
        public Point3F[] ApplyUniformSampling(IEnumerable<Point3F> points, float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = FiltersNative.ApplyUniformSampling(points_, points_.Length, radius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] filteredPoints = point3Fs.ToPoint3Fs();
            FiltersNative.Dispose(pointer);

            return filteredPoints;
        }
        #endregion

        #region # 适用体素降采样 —— Point3F[] ApplyVoxelGrid(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用体素降采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="leafSize">叶尺寸</param>
        /// <returns>过滤后点集</returns>
        public Point3F[] ApplyVoxelGrid(IEnumerable<Point3F> points, float leafSize)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = FiltersNative.ApplyVoxelGrid(points_, points_.Length, leafSize);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] filteredPoints = point3Fs.ToPoint3Fs();
            FiltersNative.Dispose(pointer);

            return filteredPoints;
        }
        #endregion

        #region # 适用近似体素降采样 —— Point3F[] ApplyApproximateVoxelGrid(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用近似体素降采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="leafSize">叶尺寸</param>
        /// <returns>过滤后点集</returns>
        public Point3F[] ApplyApproximateVoxelGrid(IEnumerable<Point3F> points, float leafSize)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = FiltersNative.ApplyApproximateVoxelGrid(points_, points_.Length, leafSize);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] filteredPoints = point3Fs.ToPoint3Fs();
            FiltersNative.Dispose(pointer);

            return filteredPoints;
        }
        #endregion

        #region # 适用统计离群点移除 —— Point3F[] ApplyStatisticalOutlierRemoval(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用统计离群点移除
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="meanK">平均距离估计的最近邻居的数量</param>
        /// <param name="stddevMult">标准差阈值系数</param>
        /// <returns>过滤后点集</returns>
        public Point3F[] ApplyStatisticalOutlierRemoval(IEnumerable<Point3F> points, int meanK, float stddevMult)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = FiltersNative.ApplyStatisticalOutlierRemoval(points_, points_.Length, meanK, stddevMult);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] filteredPoints = point3Fs.ToPoint3Fs();
            FiltersNative.Dispose(pointer);

            return filteredPoints;
        }
        #endregion

        #region # 适用半径离群点移除 —— Point3F[] ApplyRadiusOutlierRemoval(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用半径离群点移除
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="radius">搜索半径</param>
        /// <param name="minNeighborsInRadius">半径范围内点数量最小值</param>
        /// <returns>过滤后点集</returns>
        public Point3F[] ApplyRadiusOutlierRemoval(IEnumerable<Point3F> points, float radius, int minNeighborsInRadius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = FiltersNative.ApplyRadiusOutlierRemoval(points_, points_.Length, radius, minNeighborsInRadius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] filteredPoints = point3Fs.ToPoint3Fs();
            FiltersNative.Dispose(pointer);

            return filteredPoints;
        }
        #endregion
    }
}
