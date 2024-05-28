using PCLSharp.Primitives.Models;
using System.Collections.Generic;

namespace PCLSharp.Modules.Interfaces
{
    /// <summary>
    /// 点云搜索接口
    /// </summary>
    public interface ICloudSearch
    {
        #region # K近邻搜索 —— Point3F[] KSearch(IEnumerable<Point3F> points, Point3F referencePoint...
        /// <summary>
        /// K近邻搜索
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="referencePoint">参考坐标点</param>
        /// <param name="k">近邻数量</param>
        /// <returns>结果点集</returns>
        Point3F[] KSearch(IEnumerable<Point3F> points, Point3F referencePoint, int k);
        #endregion

        #region # 半径搜索 —— Point3F[] RadiusSearch(IEnumerable<Point3F> points, Point3F referencePoint...
        /// <summary>
        /// 半径搜索
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="referencePoint">参考坐标点</param>
        /// <param name="radius">搜索半径</param>
        /// <returns>结果点集</returns>
        Point3F[] RadiusSearch(IEnumerable<Point3F> points, Point3F referencePoint, float radius);
        #endregion

        #region # 八叉树搜索 —— Point3F[] OctreeSearch(IEnumerable<Point3F> points, Point3F referencePoint...
        /// <summary>
        /// 八叉树搜索
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="referencePoint">参考坐标点</param>
        /// <param name="resolution">分辨率</param>
        /// <returns>结果点集</returns>
        Point3F[] OctreeSearch(IEnumerable<Point3F> points, Point3F referencePoint, float resolution);
        #endregion
    }
}
