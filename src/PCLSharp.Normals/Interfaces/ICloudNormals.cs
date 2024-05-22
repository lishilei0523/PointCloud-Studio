using PCLSharp.Primitives.Models;
using System.Collections.Generic;

namespace PCLSharp.Normals.Interfaces
{
    /// <summary>
    /// 点云法向量接口
    /// </summary>
    public interface ICloudNormals
    {
        #region # 估算法向量 —— Normal3F[] EstimateNormalsByK(IEnumerable<Point3F> points, int k)
        /// <summary>
        /// 估算法向量
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="k">搜索近邻数量</param>
        /// <returns>法向量集</returns>
        Normal3F[] EstimateNormalsByK(IEnumerable<Point3F> points, int k);
        #endregion

        #region # 估算法向量 —— Normal3F[] EstimateNormalsByRadius(IEnumerable<Point3F> points, float radius)
        /// <summary>
        /// 估算法向量
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="radius">搜索半径</param>
        /// <returns>法向量集</returns>
        Normal3F[] EstimateNormalsByRadius(IEnumerable<Point3F> points, float radius);
        #endregion

        #region # 估算法向量 (OMP) —— Normal3F[] EstimateNormalsByKP(IEnumerable<Point3F> points, int k)
        /// <summary>
        /// 估算法向量 (OMP)
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="k">搜索近邻数量</param>
        /// <returns>法向量集</returns>
        Normal3F[] EstimateNormalsByKP(IEnumerable<Point3F> points, int k);
        #endregion

        #region # 估算法向量 (OMP) —— Normal3F[] EstimateNormalsByRadiusP(IEnumerable<Point3F> points, float radius)
        /// <summary>
        /// 估算法向量 (OMP)
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="radius">搜索半径</param>
        /// <returns>法向量集</returns>
        Normal3F[] EstimateNormalsByRadiusP(IEnumerable<Point3F> points, float radius);
        #endregion
    }
}
