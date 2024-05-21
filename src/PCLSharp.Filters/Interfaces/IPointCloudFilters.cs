using PCLSharp.Primitives.Models;
using System.Collections.Generic;

namespace PCLSharp.Filters.Interfaces
{
    /// <summary>
    /// 点云滤波接口
    /// </summary>
    public interface IPointCloudFilters
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
        /// <param name="leafSize">叶尺寸</param>
        /// <returns>过滤后点集</returns>
        Point3F[] ApplyVoxelGrid(IEnumerable<Point3F> points, float leafSize);
        #endregion

        #region # 适用离群点移除 —— Point3F[] ApplyOutlierRemoval(IEnumerable<Point3F> points...
        /// <summary>
        /// 适用离群点移除
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="meanK">平均距离估计的最近邻居的数量</param>
        /// <param name="stddevMult">标准差阈值系数</param>
        /// <returns>过滤后点集</returns>
        Point3F[] ApplyOutlierRemoval(IEnumerable<Point3F> points, int meanK, float stddevMult);
        #endregion
    }
}
