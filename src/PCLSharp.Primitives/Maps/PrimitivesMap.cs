using PCLSharp.Primitives.Models;
using System;

namespace PCLSharp.Primitives.Maps
{
    /// <summary>
    /// 基元映射
    /// </summary>
    public static class PrimitivesMap
    {
        #region # 坐标点集结构体映射坐标点数组 —— static Point3F[] ToPoint3Fs(this Point3Fs point3Fs)
        /// <summary>
        /// 坐标点集结构体映射坐标点数组
        /// </summary>
        public static unsafe Point3F[] ToPoint3Fs(this Point3Fs point3Fs)
        {
            Span<Point3F> span = new Span<Point3F>(point3Fs.Points.ToPointer(), point3Fs.Length);
            Point3F[] points = span.ToArray();

            return points;
        }
        #endregion

        #region # 法向量集结构体映射法向量数组 —— static Normal3F[] ToNormal3Fs(this Normal3Fs normal3Fs)
        /// <summary>
        /// 法向量集结构体映射法向量数组
        /// </summary>
        public static unsafe Normal3F[] ToNormal3Fs(this Normal3Fs normal3Fs)
        {
            Span<Normal3F> span = new Span<Normal3F>(normal3Fs.Normals.ToPointer(), normal3Fs.Length);
            Normal3F[] normals = span.ToArray();

            return normals;
        }
        #endregion

        #region # 坐标点法向量集结构体映射坐标点法向量数组 —— static Point3Normal3[] ToPoint3Normal3s(this Point3Normal3s...
        /// <summary>
        /// 坐标点法向量集结构体映射坐标点法向量数组
        /// </summary>
        public static unsafe Point3Normal3[] ToPoint3Normal3s(this Point3Normal3s point3Normal3s)
        {
            Span<Point3Normal3> span = new Span<Point3Normal3>(point3Normal3s.PointNormals.ToPointer(), point3Normal3s.Length);
            Point3Normal3[] pointNormals = span.ToArray();

            return pointNormals;
        }
        #endregion

        #region # 坐标点颜色集结构体映射坐标点颜色数组 —— static Point3Color4[] ToPoint3Color4s(this Point3Color4s...
        /// <summary>
        /// 坐标点颜色集结构体映射坐标点颜色数组
        /// </summary>
        public static unsafe Point3Color4[] ToPoint3Color4s(this Point3Color4s point3Color4s)
        {
            Span<Point3Color4> span = new Span<Point3Color4>(point3Color4s.PointColors.ToPointer(), point3Color4s.Length);
            Point3Color4[] pointColors = span.ToArray();

            return pointColors;
        }
        #endregion
    }
}
