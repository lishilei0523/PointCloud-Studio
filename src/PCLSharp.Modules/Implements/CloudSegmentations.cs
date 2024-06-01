using PCLSharp.Modules.Declarations;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Implements
{
    /// <summary>
    /// 点云分割实现
    /// </summary>
    public class CloudSegmentations : ICloudSegmentations
    {
        #region # 分割平面 —— Point3F[] SegmentPlane(IEnumerable<Point3F> points...
        /// <summary>
        /// 分割平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="optimizeCoefficients">是否优化模型系数</param>
        /// <param name="probability">概率</param>
        /// <param name="distanceThreshold">距离阈值</param>
        /// <param name="maxIterationsCount">最大迭代次数</param>
        /// <param name="a">平面方程系数a</param>
        /// <param name="b">平面方程系数b</param>
        /// <param name="c">平面方程系数c</param>
        /// <param name="d">平面方程系数d</param>
        /// <returns>平面点云</returns>
        public Point3F[] SegmentPlane(IEnumerable<Point3F> points, bool optimizeCoefficients, float probability, float distanceThreshold, int maxIterationsCount, out int a, out int b, out int c, out int d)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                a = b = c = d = 0;
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = SegmentationsNative.SegmentPlane(points_, points_.Length, optimizeCoefficients, probability, distanceThreshold, maxIterationsCount, out a, out b, out c, out d);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] segmentedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return segmentedPoints;
        }
        #endregion

        #region # 分割球体 —— Point3F[] SegmentSphere(IEnumerable<Point3F> points...
        /// <summary>
        /// 分割球体
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="optimizeCoefficients">是否优化模型系数</param>
        /// <param name="probability">概率</param>
        /// <param name="distanceThreshold">距离阈值</param>
        /// <param name="minRadius">球体最小半径</param>
        /// <param name="maxRadius">球体最大半径</param>
        /// <param name="maxIterationsCount">最大迭代次数</param>
        /// <param name="center">球心</param>
        /// <param name="radius">半径</param>
        /// <returns>球体点云</returns>
        public Point3F[] SegmentSphere(IEnumerable<Point3F> points, bool optimizeCoefficients, float probability, float distanceThreshold, float minRadius, float maxRadius, int maxIterationsCount, out Point3F center, out float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                center = new Point3F();
                radius = 0;
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = SegmentationsNative.SegmentSphere(points_, points_.Length, optimizeCoefficients, probability, distanceThreshold, minRadius, maxRadius, maxIterationsCount, out center, out radius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] segmentedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return segmentedPoints;
        }
        #endregion

        #region # 欧几里得聚类分割 —— Point3F[][] EuclidClusterSegment(IEnumerable<Point3F> points...
        /// <summary>
        /// 欧几里得聚类分割
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="clusterTolerance">簇搜索容差</param>
        /// <param name="minClusterSize">簇最小尺寸</param>
        /// <param name="maxClusterSize">簇最大尺寸</param>
        /// <returns>点云簇列表</returns>
        public unsafe Point3F[][] EuclidClusterSegment(IEnumerable<Point3F> points, float clusterTolerance, int minClusterSize, int maxClusterSize)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F[]>();
            }

            #endregion

            IntPtr pointer = SegmentationsNative.EuclidClusterSegment(points_, points_.Length, clusterTolerance, minClusterSize, maxClusterSize, out int clustersCount);
            Point3F[][] pointsGroup = new Point3F[clustersCount][];

            Point3Fs** pointsGroupPtr = (Point3Fs**)pointer.ToPointer();
            for (int clusterIndex = 0; clusterIndex < clustersCount; clusterIndex++)
            {
                Point3Fs* pointsPtr = pointsGroupPtr[clusterIndex];
                Span<Point3F> span = new Span<Point3F>(pointsPtr->Points.ToPointer(), pointsPtr->Length);
                pointsGroup[clusterIndex] = span.ToArray();
            }
            DisposeNative.DisposePoint3FsGroup(pointer, clustersCount);

            return pointsGroup;
        }
        #endregion

        #region # 区域生长分割 —— Point3F[][] RegionGrowingSegment(IEnumerable<Point3F> points...
        /// <summary>
        /// 区域生长分割
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="clusterK">簇K</param>
        /// <param name="smoothnessThreshold">平滑阈值（角度）</param>
        /// <param name="curvatureThreshold">曲率阈值</param>
        /// <param name="minClusterSize">簇最小尺寸</param>
        /// <param name="maxClusterSize">簇最大尺寸</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>点云簇列表</returns>
        public unsafe Point3F[][] RegionGrowingSegment(IEnumerable<Point3F> points, int normalK, int clusterK, float smoothnessThreshold, float curvatureThreshold, int minClusterSize, int maxClusterSize, int threadsCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F[]>();
            }

            #endregion

            IntPtr pointer = SegmentationsNative.RegionGrowingSegment(points_, points_.Length, normalK, clusterK, smoothnessThreshold, curvatureThreshold, minClusterSize, maxClusterSize, threadsCount, out int clustersCount);
            Point3F[][] pointsGroup = new Point3F[clustersCount][];

            Point3Fs** pointsGroupPtr = (Point3Fs**)pointer.ToPointer();
            for (int clusterIndex = 0; clusterIndex < clustersCount; clusterIndex++)
            {
                Point3Fs* pointsPtr = pointsGroupPtr[clusterIndex];
                Span<Point3F> span = new Span<Point3F>(pointsPtr->Points.ToPointer(), pointsPtr->Length);
                pointsGroup[clusterIndex] = span.ToArray();
            }
            DisposeNative.DisposePoint3FsGroup(pointer, clustersCount);

            return pointsGroup;
        }
        #endregion

        #region # 区域生长颜色分割 —— Point3Color4[][] RegionGrowingColorSegment(IEnumerable<Point3Color4> points...
        /// <summary>
        /// 区域生长颜色分割
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="clusterK">簇K</param>
        /// <param name="distanceThreshold">距离阈值</param>
        /// <param name="smoothnessThreshold">平滑阈值（角度）</param>
        /// <param name="curvatureThreshold">曲率阈值</param>
        /// <param name="pointColorThreshold">点颜色阈值</param>
        /// <param name="regionColorThreshold">区域颜色阈值</param>
        /// <param name="minClusterSize">簇最小尺寸</param>
        /// <param name="maxClusterSize">簇最大尺寸</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>点云簇列表</returns>
        public unsafe Point3Color4[][] RegionGrowingColorSegment(IEnumerable<Point3Color4> points, int normalK, int clusterK, float distanceThreshold, float smoothnessThreshold, float curvatureThreshold, float pointColorThreshold, float regionColorThreshold, int minClusterSize, int maxClusterSize, int threadsCount)
        {
            Point3Color4[] points_ = points?.ToArray() ?? Array.Empty<Point3Color4>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3Color4[]>();
            }

            #endregion

            IntPtr pointer = SegmentationsNative.RegionGrowingColorSegment(points_, points_.Length, normalK, clusterK, distanceThreshold, smoothnessThreshold, curvatureThreshold, pointColorThreshold, regionColorThreshold, minClusterSize, maxClusterSize, threadsCount, out int clustersCount);
            Point3Color4[][] pointsGroup = new Point3Color4[clustersCount][];

            Point3Color4s** pointsGroupPtr = (Point3Color4s**)pointer.ToPointer();
            for (int clusterIndex = 0; clusterIndex < clustersCount; clusterIndex++)
            {
                Point3Color4s* pointsPtr = pointsGroupPtr[clusterIndex];
                Span<Point3Color4> span = new Span<Point3Color4>(pointsPtr->PointColors.ToPointer(), pointsPtr->Length);
                pointsGroup[clusterIndex] = span.ToArray();
            }
            DisposeNative.DisposePoint3Color4sGroup(pointer, clustersCount);

            return pointsGroup;
        }
        #endregion
    }
}
