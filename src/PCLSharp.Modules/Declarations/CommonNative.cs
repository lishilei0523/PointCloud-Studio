using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云通用操作声明
    /// </summary>
    internal static class CommonNative
    {
        #region # 估算质心 —— static extern IntPtr EstimateCentroid(Point3F[] points...
        /// <summary>
        /// 估算质心
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <returns>质心坐标点</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "estimateCentroid")]
        public static extern IntPtr EstimateCentroid(Point3F[] points, int length);
        #endregion

        #region # 仿射变换 —— static extern IntPtr AffineTransform(Point3F[] points...
        /// <summary>
        /// 仿射变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="pose">位姿</param>
        /// <returns>变换后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "affineTransform")]
        public static extern IntPtr AffineTransform(Point3F[] points, int length, Pose pose);
        #endregion

        #region # 矩阵变换 —— static extern IntPtr MatrixTransform(Point3F[] points...
        /// <summary>
        /// 矩阵变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="matrixArray">矩阵数组(长度: 16)</param>
        /// <returns>变换后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "matrixTransform")]
        public static extern IntPtr MatrixTransform(Point3F[] points, int length, float[] matrixArray);
        #endregion

        #region # 合并坐标点法向量 —— static extern IntPtr MergePointsNormals(Point3F[] points...
        /// <summary>
        /// 合并坐标点法向量
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normal3Fs">法向量集</param>
        /// <param name="length">点集长度</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "mergePointsNormals")]
        public static extern IntPtr MergePointsNormals(Point3F[] points, Normal3F[] normal3Fs, int length);
        #endregion

        #region # 长方体剪裁 —— static extern IntPtr CropBox(Point3F[] points...
        /// <summary>
        /// 长方体剪裁
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="minPoint">最小坐标点</param>
        /// <param name="maxPoint">最大坐标点</param>
        /// <param name="negative">true: 剪裁/false: 保留</param>
        /// <returns>剪裁后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "cropBox")]
        public static extern IntPtr CropBox(Point3F[] points, int length, Point3F minPoint, Point3F maxPoint, bool negative);
        #endregion

        #region # 凸包剪裁 —— static extern IntPtr CropConvexHull(Point3F[] points...
        /// <summary>
        /// 凸包剪裁
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="contourPoints">轮廓点集</param>
        /// <param name="contourLength">轮廓点集长度</param>
        /// <param name="dimensionsCount">维度数</param>
        /// <returns>剪裁后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "cropConvexHull")]
        public static extern IntPtr CropConvexHull(Point3F[] points, int length, Point3F[] contourPoints, int contourLength, int dimensionsCount);
        #endregion

        #region # 投射平面 —— static extern IntPtr ProjectPlane(Point3F[] points...
        /// <summary>
        /// 投射平面
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="a">平面方程系数a</param>
        /// <param name="b">平面方程系数b</param>
        /// <param name="c">平面方程系数c</param>
        /// <param name="d">平面方程系数d</param>
        /// <returns>投射后点云</returns>
        /// <remarks>平面方程: ax + by +cz + d = 0</remarks>
        [DllImport(AssemblyNames.Modules, EntryPoint = "projectPlane")]
        public static extern IntPtr ProjectPlane(Point3F[] points, int length, float a, float b, float c, float d);
        #endregion

        #region # 提取边框 —— static extern IntPtr ExtractBorder(Point3F[] points...
        /// <summary>
        /// 提取边框
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <returns>边框点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "extractBorder")]
        public static extern IntPtr ExtractBorder(Point3F[] points, int length);
        #endregion

        #region # 提取边界 —— static extern IntPtr ExtractBoundary(Point3F[] points...
        /// <summary>
        /// 提取边界
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="featureRadius">特征半径</param>
        /// <param name="angleThreshold">角度阈值</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>边界点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "extractBoundary")]
        public static extern IntPtr ExtractBoundary(Point3F[] points, int length, int normalK, float featureRadius, float angleThreshold, int threadsCount);
        #endregion
    }
}
