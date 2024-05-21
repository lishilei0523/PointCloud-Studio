using PCLSharp.Filters.Declarations;
using PCLSharp.Filters.Interfaces;
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
        public unsafe Point3F[] ApplyPassThrogh(IEnumerable<Point3F> points, string axis, float limitMin, float limixMax)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr point3FsPtr = FiltersNative.ApplyPassThrogh(points_, points_.Length, axis, limitMin, limixMax);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(point3FsPtr);
            Span<Point3F> span = new Span<Point3F>(point3Fs.Points.ToPointer(), point3Fs.Length);
            Point3F[] filteredPoints = span.ToArray();
            FiltersNative.Dispose(point3FsPtr);

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
        public unsafe Point3F[] ApplyUniformSampling(IEnumerable<Point3F> points, float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr point3FsPtr = FiltersNative.ApplyUniformSampling(points_, points_.Length, radius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(point3FsPtr);
            Span<Point3F> span = new Span<Point3F>(point3Fs.Points.ToPointer(), point3Fs.Length);
            Point3F[] filteredPoints = span.ToArray();
            FiltersNative.Dispose(point3FsPtr);

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
        public unsafe Point3F[] ApplyVoxelGrid(IEnumerable<Point3F> points, float leafSize)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr point3FsPtr = FiltersNative.ApplyVoxelGrid(points_, points_.Length, leafSize);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(point3FsPtr);
            Span<Point3F> span = new Span<Point3F>(point3Fs.Points.ToPointer(), point3Fs.Length);
            Point3F[] filteredPoints = span.ToArray();
            FiltersNative.Dispose(point3FsPtr);

            return filteredPoints;
        }
        #endregion

        #region # 适用离群点移除 —— Point3F[] ApplyOutlierRemoval(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用离群点移除
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="meanK">平均距离估计的最近邻居的数量</param>
        /// <param name="stddevMult">标准差阈值系数</param>
        /// <returns>过滤后点集</returns>
        public unsafe Point3F[] ApplyOutlierRemoval(IEnumerable<Point3F> points, int meanK, float stddevMult)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr point3FsPtr = FiltersNative.ApplyOutlierRemoval(points_, points_.Length, meanK, stddevMult);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(point3FsPtr);
            Span<Point3F> span = new Span<Point3F>(point3Fs.Points.ToPointer(), point3Fs.Length);
            Point3F[] filteredPoints = span.ToArray();
            FiltersNative.Dispose(point3FsPtr);

            return filteredPoints;
        }
        #endregion
    }
}
