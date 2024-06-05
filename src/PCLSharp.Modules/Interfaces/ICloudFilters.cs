using PCLSharp.Primitives.Models;
using System.Collections.Generic;

namespace PCLSharp.Modules.Interfaces
{
    /// <summary>
    /// 点云文件接口
    /// </summary>
    public interface ICloudFilters
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
        Point3F[] ApplyPassThrogh(IEnumerable<Point3F> points, string axis, float limitMin, float limixMax);
        #endregion

        #region # 适用随机采样 —— Point3F[] ApplyRandomSampling(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用随机采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="seed">随机种子</param>
        /// <param name="samplesCount">采样数量</param>
        /// <returns>过滤后点集</returns>
        Point3F[] ApplyRandomSampling(IEnumerable<Point3F> points, int seed, int samplesCount);
        #endregion

        #region # 适用均匀采样 —— Point3F[] ApplyUniformSampling(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用均匀采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="radius">采样半径</param>
        /// <returns>过滤后点集</returns>
        Point3F[] ApplyUniformSampling(IEnumerable<Point3F> points, float radius);
        #endregion

        #region # 适用体素降采样 —— Point3F[] ApplyVoxelGrid(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用体素降采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="leafSize">网格尺寸</param>
        /// <returns>过滤后点集</returns>
        Point3F[] ApplyVoxelGrid(IEnumerable<Point3F> points, float leafSize);
        #endregion

        #region # 适用近似体素降采样 —— Point3F[] ApplyApproxVoxelGrid(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用近似体素降采样
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="leafSize">网格尺寸</param>
        /// <returns>过滤后点集</returns>
        Point3F[] ApplyApproxVoxelGrid(IEnumerable<Point3F> points, float leafSize);
        #endregion

        #region # 适用统计离群点移除 —— Point3F[] ApplyStatisticalOutlierRemoval(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用统计离群点移除
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="meanK">平均距离估计的最近邻居的数量</param>
        /// <param name="stddevMult">标准差阈值系数</param>
        /// <returns>过滤后点集</returns>
        Point3F[] ApplyStatisticalOutlierRemoval(IEnumerable<Point3F> points, int meanK, float stddevMult);
        #endregion

        #region # 适用半径离群点移除 —— Point3F[] ApplyRadiusOutlierRemoval(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用半径离群点移除
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="radius">搜索半径</param>
        /// <param name="minNeighborsInRadius">半径范围内点数量最小值</param>
        /// <returns>过滤后点集</returns>
        Point3F[] ApplyRadiusOutlierRemoval(IEnumerable<Point3F> points, float radius, int minNeighborsInRadius);
        #endregion
    }
}
