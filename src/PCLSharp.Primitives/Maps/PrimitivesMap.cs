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

        #region # 坐标点集结构体映射坐标点数组 —— static Point3FI[] ToPoint3FIs(this Point3FIs point3FIs)
        /// <summary>
        /// 坐标点集结构体映射坐标点数组
        /// </summary>
        public static unsafe Point3FI[] ToPoint3FIs(this Point3FIs point3FIs)
        {
            Span<Point3FI> span = new Span<Point3FI>(point3FIs.Points.ToPointer(), point3FIs.Length);
            Point3FI[] points = span.ToArray();

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

        #region # RGB颜色集结构体映射RGB颜色数组 —— static Color3F[] ToColor3Fs(this Color3Fs color3Fs)
        /// <summary>
        /// RGB颜色集结构体映射RGB颜色数组
        /// </summary>
        public static unsafe Color3F[] ToColor3Fs(this Color3Fs color3Fs)
        {
            Span<Color3F> span = new Span<Color3F>(color3Fs.Colors.ToPointer(), color3Fs.Length);
            Color3F[] colors = span.ToArray();

            return colors;
        }
        #endregion

        #region # RGBA颜色集结构体映射RGBA颜色数组 —— static Color4F[] ToColor4Fs(this Color4Fs color4Fs)
        /// <summary>
        /// RGBA颜色集结构体映射RGBA颜色数组
        /// </summary>
        public static unsafe Color4F[] ToColor3Fs(this Color4Fs color4Fs)
        {
            Span<Color4F> span = new Span<Color4F>(color4Fs.Colors.ToPointer(), color4Fs.Length);
            Color4F[] colors = span.ToArray();

            return colors;
        }
        #endregion
    }
}
