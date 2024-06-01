using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云分割声明
    /// </summary>
    internal static class SegmentationsNative
    {
        #region # 分割平面 —— static extern IntPtr SegmentPlane(Point3F[] points...
        /// <summary>
        /// 分割平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="optimizeCoefficients">是否优化模型系数</param>
        /// <param name="probability">概率</param>
        /// <param name="distanceThreshold">距离阈值</param>
        /// <param name="maxIterationsCount">最大迭代次数</param>
        /// <param name="a">平面方程系数a</param>
        /// <param name="b">平面方程系数b</param>
        /// <param name="c">平面方程系数c</param>
        /// <param name="d">平面方程系数d</param>
        /// <returns>平面点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "segmentPlane")]
        public static extern IntPtr SegmentPlane(Point3F[] points, int length, bool optimizeCoefficients, float probability, float distanceThreshold, int maxIterationsCount, out int a, out int b, out int c, out int d);
        #endregion

        #region # 分割球体 —— static extern IntPtr SegmentSphere(Point3F[] points...
        /// <summary>
        /// 分割球体
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="optimizeCoefficients">是否优化模型系数</param>
        /// <param name="probability">概率</param>
        /// <param name="distanceThreshold">距离阈值</param>
        /// <param name="minRadius">球体最小半径</param>
        /// <param name="maxRadius">球体最大半径</param>
        /// <param name="maxIterationsCount">最大迭代次数</param>
        /// <param name="center">球心</param>
        /// <param name="radius">半径</param>
        /// <returns>球体点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "segmentSphere")]
        public static extern IntPtr SegmentSphere(Point3F[] points, int length, bool optimizeCoefficients, float probability, float distanceThreshold, float minRadius, float maxRadius, int maxIterationsCount, out Point3F center, out float radius);
        #endregion

        #region # 欧几里得聚类分割 —— static extern IntPtr EuclidClusterSegment(Point3F[] points...
        /// <summary>
        /// 欧几里得聚类分割
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="clusterTolerance">簇搜索容差</param>
        /// <param name="minClusterSize">簇最小尺寸</param>
        /// <param name="maxClusterSize">簇最大尺寸</param>
        /// <param name="clustersCount">点云簇数</param>
        /// <returns>点云簇列表</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "euclidClusterSegment")]
        public static extern IntPtr EuclidClusterSegment(Point3F[] points, int length, float clusterTolerance, int minClusterSize, int maxClusterSize, out int clustersCount);
        #endregion

        #region # 区域生长分割 —— static extern IntPtr RegionGrowingSegment(Point3F[] points...
        /// <summary>
        /// 区域生长分割
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="clusterK">簇K</param>
        /// <param name="smoothnessThreshold">平滑阈值（角度）</param>
        /// <param name="curvatureThreshold">曲率阈值</param>
        /// <param name="minClusterSize">簇最小尺寸</param>
        /// <param name="maxClusterSize">簇最大尺寸</param>
        /// <param name="threadsCount">线程数</param>
        /// <param name="clustersCount">点云簇数</param>
        /// <returns>点云簇列表</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "regionGrowingSegment")]
        public static extern IntPtr RegionGrowingSegment(Point3F[] points, int length, int normalK, int clusterK, float smoothnessThreshold, float curvatureThreshold, int minClusterSize, int maxClusterSize, int threadsCount, out int clustersCount);
        #endregion

        #region # 区域生长颜色分割 —— static extern IntPtr RegionGrowingColorSegment(Point3Color4[] points...
        /// <summary>
        /// 区域生长颜色分割
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
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
        /// <param name="clustersCount">点云簇数</param>
        /// <returns>点云簇列表</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "regionGrowingColorSegment")]
        public static extern IntPtr RegionGrowingColorSegment(Point3Color4[] points, int length, int normalK, int clusterK, float distanceThreshold, float smoothnessThreshold, float curvatureThreshold, float pointColorThreshold, float regionColorThreshold, int minClusterSize, int maxClusterSize, int threadsCount, out int clustersCount);
        #endregion
    }
}
