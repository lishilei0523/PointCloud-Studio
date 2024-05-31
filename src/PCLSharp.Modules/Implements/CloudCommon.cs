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
    /// 点云通用操作实现
    /// </summary>
    public class CloudCommon : ICloudCommon
    {
        #region # 估算质心 —— Point3F EstimateCentroid(IEnumerable<Point3F> points)
        /// <summary>
        /// 估算质心
        /// </summary>
        /// <param name="points">点集</param>
        /// <returns>质心坐标点</returns>
        public Point3F EstimateCentroid(IEnumerable<Point3F> points)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return new Point3F();
            }

            #endregion

            IntPtr pointer = CommonNative.EstimateCentroid(points_, points_.Length);
            Point3F centroid = Marshal.PtrToStructure<Point3F>(pointer);
            DisposeNative.DisposePoint3F(pointer);

            return centroid;
        }
        #endregion

        #region # 仿射变换 —— Point3F[] AffineTransform(IEnumerable<Point3F> points, Pose pose)
        /// <summary>
        /// 仿射变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="pose">位姿</param>
        /// <returns>变换后点云</returns>
        public Point3F[] AffineTransform(IEnumerable<Point3F> points, Pose pose)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.AffineTransform(points_, points_.Length, pose);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] transformedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return transformedPoints;
        }
        #endregion

        #region # 矩阵变换 —— Point3F[] MatrixTransform(IEnumerable<Point3F> points...
        /// <summary>
        /// 矩阵变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="matrixArray">矩阵数组(长度: 16)</param>
        /// <returns>变换后点云</returns>
        public Point3F[] MatrixTransform(IEnumerable<Point3F> points, float[] matrixArray)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.MatrixTransform(points_, points_.Length, matrixArray);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] transformedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return transformedPoints;
        }
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
        public Point3F[] CropBox(IEnumerable<Point3F> points, Point3F minPoint, Point3F maxPoint, bool negative)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.CropBox(points_, points_.Length, minPoint, maxPoint, negative);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] croppedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return croppedPoints;
        }
        #endregion

        #region # 凸包剪裁 —— Point3F[] CropConvexHull(IEnumerable<Point3F> points...
        /// <summary>
        /// 凸包剪裁
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="contourPoints">轮廓点集</param>
        /// <param name="dimensionsCount">维度数</param>
        /// <returns>剪裁后点云</returns>
        public Point3F[] CropConvexHull(IEnumerable<Point3F> points, IEnumerable<Point3F> contourPoints, int dimensionsCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] contourPoints_ = contourPoints?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }
            if (!contourPoints_.Any())
            {
                return points_;
            }

            #endregion

            IntPtr pointer = CommonNative.CropConvexHull(points_, points_.Length, contourPoints_, contourPoints_.Length, dimensionsCount);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] croppedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return croppedPoints;
        }
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
        public Point3F[] ProjectPlane(IEnumerable<Point3F> points, float a, float b, float c, float d)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ProjectPlane(points_, points_.Length, a, b, c, d);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] projectedPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return projectedPoints;
        }
        #endregion

        #region # 提取边框 —— Point3F[] ExtractBorder(IEnumerable<Point3F> points)
        /// <summary>
        /// 提取边框
        /// </summary>
        /// <param name="points">点集</param>
        /// <returns>边框点云</returns>
        public Point3F[] ExtractBorder(IEnumerable<Point3F> points)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ExtractBorder(points_, points_.Length);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] borderPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return borderPoints;
        }
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
        public Point3F[] ExtractBoundary(IEnumerable<Point3F> points, int normalK, float featureRadius, float angleThreshold, int threadsCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = CommonNative.ExtractBoundary(points_, points_.Length, normalK, featureRadius, angleThreshold, threadsCount);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] boundaryPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return boundaryPoints;
        }
        #endregion
    }
}
