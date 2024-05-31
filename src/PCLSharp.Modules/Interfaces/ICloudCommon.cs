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

        #region # 矩阵变换 —— Point3F[] MatrixTransform(IEnumerable<Point3F> points...
        /// <summary>
        /// 矩阵变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="matrixArray">矩阵数组(长度: 16)</param>
        /// <returns>变换后点云</returns>
        Point3F[] MatrixTransform(IEnumerable<Point3F> points, float[] matrixArray);
        #endregion

        #region # 长方体剪裁 —— Point3F[] CropBox(IEnumerable<Point3F> points...
        /// <summary>
        /// 长方体剪裁
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="minPoint">最小坐标点</param>
        /// <param name="maxPoint">最大坐标点</param>
        /// <param name="negative">true: 剪裁/false: 保留</param>
        /// <returns>剪裁后点云</returns>
        Point3F[] CropBox(IEnumerable<Point3F> points, Point3F minPoint, Point3F maxPoint, bool negative);
        #endregion

        #region # 凸包剪裁 —— Point3F[] CropConvexHull(IEnumerable<Point3F> points...
        /// <summary>
        /// 凸包剪裁
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="contourPoints">轮廓点集</param>
        /// <param name="dimensionsCount">维度数</param>
        /// <returns>剪裁后点云</returns>
        Point3F[] CropConvexHull(IEnumerable<Point3F> points, IEnumerable<Point3F> contourPoints, int dimensionsCount);
        #endregion

        #region # 投射平面 —— Point3F[] ProjectPlane(IEnumerable<Point3F> points...
        /// <summary>
        /// 投射平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="a">平面方程系数a</param>
        /// <param name="b">平面方程系数b</param>
        /// <param name="c">平面方程系数c</param>
        /// <param name="d">平面方程系数d</param>
        /// <returns>投射后点云</returns>
        /// <remarks>平面方程: ax + by +cz + d = 0</remarks>
        Point3F[] ProjectPlane(IEnumerable<Point3F> points, float a, float b, float c, float d);
        #endregion

        #region # 提取边框 —— Point3F[] ExtractBorder(IEnumerable<Point3F> points)
        /// <summary>
        /// 提取边框
        /// </summary>
        /// <param name="points">点集</param>
        /// <returns>边框点云</returns>
        Point3F[] ExtractBorder(IEnumerable<Point3F> points);
        #endregion

        #region # 提取边界 —— Point3F[] ExtractBoundary(IEnumerable<Point3F> points...
        /// <summary>
        /// 提取边界
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="featureRadius">特征半径</param>
        /// <param name="angleThreshold">角度阈值</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>边界点云</returns>
        Point3F[] ExtractBoundary(IEnumerable<Point3F> points, int normalK, float featureRadius, float angleThreshold, int threadsCount);
        #endregion
    }
}
