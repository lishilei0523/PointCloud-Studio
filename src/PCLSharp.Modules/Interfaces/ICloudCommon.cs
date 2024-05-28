using PCLSharp.Primitives.Models;
using System.Collections.Generic;

namespace PCLSharp.Modules.Interfaces
{
    /// <summary>
    /// 点云通用操作接口
    /// </summary>
    public interface ICloudCommon
    {
        #region # 估算质心 —— Point3F EstimateCentroid(IEnumerable<Point3F> points)
        /// <summary>
        /// 估算质心
        /// </summary>
        /// <param name="points">点集</param>
        /// <returns>质心坐标点</returns>
        Point3F EstimateCentroid(IEnumerable<Point3F> points);
        #endregion

        #region # 仿射变换 —— Point3F[] AffineTransform(IEnumerable<Point3F> points, Pose pose)
        /// <summary>
        /// 仿射变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="pose">位姿</param>
        /// <returns>变换后点云</returns>
        Point3F[] AffineTransform(IEnumerable<Point3F> points, Pose pose);
        #endregion
    }
}
