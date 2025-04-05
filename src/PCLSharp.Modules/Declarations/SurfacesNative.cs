using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云表面声明
    /// </summary>
    internal static class SurfacesNative
    {
        #region # 适用贪婪投影三角化 —— static extern IntPtr ApplyGreedyProjection(Point3F[] points...
        /// <summary>
        /// 适用贪婪投影三角化
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="searchRadius">搜索半径</param>
        /// <param name="mu">近邻点最远倍数</param>
        /// <param name="maxNearestNeighbors">最多邻域数</param>
        /// <param name="maxSurfaceAngle">偏离法向量最大角度</param>
        /// <param name="minAngle">三角形最小角度</param>
        /// <param name="maxAngle">三角形最大角度</param>
        /// <param name="normalConsistency">保证法向量朝向一致</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>网格几何</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "applyGreedyProjection")]
        public static extern IntPtr ApplyGreedyProjection(Point3F[] points, int length, int normalK, float searchRadius, float mu, int maxNearestNeighbors, double maxSurfaceAngle, double minAngle, double maxAngle, bool normalConsistency, int threadsCount);
        #endregion

        #region # 适用泊松重建 —— static extern IntPtr ApplyPoissonReconstruction(Point3F[] points...
        /// <summary>
        /// 适用泊松重建
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="confidence">是否使用法向量置信</param>
        /// <param name="degree">度数</param>
        /// <param name="depth">树最大深度</param>
        /// <param name="isoDivide">ISO等值面深度</param>
        /// <param name="manifold">是否添加多边形重心</param>
        /// <param name="outputPolygons">是否输出多边形网格</param>
        /// <param name="samplesPerNode">八叉树样本点最小数量</param>
        /// <param name="scale">重构立方体与样本边界立方体直径比率</param>
        /// <param name="solverDivide">Gauss-Seidel迭代深度</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>网格几何</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "applyPoissonReconstruction")]
        public static extern IntPtr ApplyPoissonReconstruction(Point3F[] points, int length, int normalK, bool confidence, int degree, int depth, int isoDivide, bool manifold, bool outputPolygons, float samplesPerNode, float scale, int solverDivide, int threadsCount);
        #endregion

        #region # 适用移动立方体重建 —— static extern IntPtr ApplyMarchingCubes(Point3F[] points...
        /// <summary>
        /// 适用移动立方体重建
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="isoLevel">等值面值</param>
        /// <param name="gridResolution">网格分辨率</param>
        /// <param name="percentageExtendGrid">自由空间比例</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>网格几何</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "applyMarchingCubes")]
        public static extern IntPtr ApplyMarchingCubes(Point3F[] points, int length, int normalK, float isoLevel, int gridResolution, float percentageExtendGrid, int threadsCount);
        #endregion
    }
}
