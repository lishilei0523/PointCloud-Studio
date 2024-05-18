using Sample.Presentation.Declares;
using Sample.Presentation.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;

namespace Sample.Presentation.Presenters
{
    /// <summary>
    /// 点云呈现器
    /// </summary>
    public class PointCloudPresenter
    {
        #region # 适用直通滤波 —— ICollection<Point3F> ApplyPassThrogh(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用直通滤波
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="axis">过滤坐标轴</param>
        /// <param name="limitMin">过滤范围最小值</param>
        /// <param name="limixMax">过滤范围最大值</param>
        /// <returns>过滤后点集</returns>
        public unsafe ICollection<Point3F> ApplyPassThrogh(IEnumerable<Point3F> points, string axis, float limitMin, float limixMax)
        {
            Point3F[] point3Fs = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!point3Fs.Any())
            {
                return new List<Point3F>();
            }

            #endregion

            IntPtr pointArrayPtr = FilterDeclare.ApplyPassThrogh(point3Fs, point3Fs.Length, axis, limitMin, limixMax);
            PointArray pointArray = Marshal.PtrToStructure<PointArray>(pointArrayPtr);
            Span<Point3F> sampledPoints = new Span<Point3F>(pointArray.Points.ToPointer(), pointArray.Length);
            FilterDeclare.Dispose(pointArrayPtr);

            return sampledPoints.ToArray();
        }
        #endregion

        #region # 适用均匀采样 —— ICollection<Point3F> ApplyUniformSampling(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用均匀采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="radius">采样半径</param>
        /// <returns>过滤后点集</returns>
        public unsafe ICollection<Point3F> ApplyUniformSampling(IEnumerable<Point3F> points, float radius)
        {
            Point3F[] point3Fs = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!point3Fs.Any())
            {
                return new List<Point3F>();
            }

            #endregion

            IntPtr pointArrayPtr = FilterDeclare.ApplyUniformSampling(point3Fs, point3Fs.Length, radius);
            PointArray pointArray = Marshal.PtrToStructure<PointArray>(pointArrayPtr);
            Span<Point3F> span = new Span<Point3F>(pointArray.Points.ToPointer(), pointArray.Length);
            Point3F[] sampledPoints = span.ToArray();
            FilterDeclare.Dispose(pointArrayPtr);

            return sampledPoints;
        }
        #endregion

        #region # 适用体素降采样 —— ICollection<Point3F> ApplyVoxelGrid(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用体素降采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="leafSize">叶尺寸</param>
        /// <returns>过滤后点集</returns>
        public unsafe ICollection<Point3F> ApplyVoxelGrid(IEnumerable<Point3F> points, float leafSize)
        {
            Point3F[] point3Fs = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!point3Fs.Any())
            {
                return new List<Point3F>();
            }

            #endregion

            IntPtr pointArrayPtr = FilterDeclare.ApplyVoxelGrid(point3Fs, point3Fs.Length, leafSize);
            PointArray pointArray = Marshal.PtrToStructure<PointArray>(pointArrayPtr);
            Span<Point3F> span = new Span<Point3F>(pointArray.Points.ToPointer(), pointArray.Length);
            Point3F[] sampledPoints = span.ToArray();
            FilterDeclare.Dispose(pointArrayPtr);

            return sampledPoints;
        }
        #endregion

        #region # 适用离群点移除 —— ICollection<Point3F> ApplyOutlierRemoval(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用离群点移除
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="meanK">平均距离估计的最近邻居的数量</param>
        /// <param name="stddevMult">标准差阈值系数</param>
        /// <returns>过滤后点集</returns>
        public unsafe ICollection<Point3F> ApplyOutlierRemoval(IEnumerable<Point3F> points, int meanK, float stddevMult)
        {
            Point3F[] point3Fs = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!point3Fs.Any())
            {
                return new List<Point3F>();
            }

            #endregion

            IntPtr pointArrayPtr = FilterDeclare.ApplyOutlierRemoval(point3Fs, point3Fs.Length, meanK, stddevMult);
            PointArray pointArray = Marshal.PtrToStructure<PointArray>(pointArrayPtr);
            Span<Point3F> span = new Span<Point3F>(pointArray.Points.ToPointer(), pointArray.Length);
            Point3F[] sampledPoints = span.ToArray();
            FilterDeclare.Dispose(pointArrayPtr);

            return sampledPoints;
        }
        #endregion
    }
}
