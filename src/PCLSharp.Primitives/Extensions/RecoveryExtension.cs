using PCLSharp.Primitives.Features;
using PCLSharp.Primitives.Models;
using System;

namespace PCLSharp.Primitives.Extensions
{
    /// <summary>
    /// 重建扩展
    /// </summary>
    public static unsafe class RecoveryExtension
    {
        #region # 坐标点集重建坐标点数组 —— static Point3F[] Recover(this Point3Fs point3Fs)
        /// <summary>
        /// 坐标点集重建坐标点数组
        /// </summary>
        public static Point3F[] Recover(this Point3Fs point3Fs)
        {
            Span<Point3F> span = new Span<Point3F>(point3Fs.Points.ToPointer(), point3Fs.Length);
            Point3F[] points = span.ToArray();

            return points;
        }
        #endregion

        #region # 法向量集重建法向量数组 —— static Normal3F[] Recover(this Normal3Fs normal3Fs)
        /// <summary>
        /// 法向量集重建法向量数组
        /// </summary>
        public static Normal3F[] Recover(this Normal3Fs normal3Fs)
        {
            Span<Normal3F> span = new Span<Normal3F>(normal3Fs.Normals.ToPointer(), normal3Fs.Length);
            Normal3F[] normals = span.ToArray();

            return normals;
        }
        #endregion

        #region # 坐标点法向量集重建坐标点法向量数组 —— static Point3Normal3[] Recover(this Point3Normal3s...
        /// <summary>
        /// 坐标点法向量集重建坐标点法向量数组
        /// </summary>
        public static Point3Normal3[] Recover(this Point3Normal3s point3Normal3s)
        {
            Span<Point3Normal3> span = new Span<Point3Normal3>(point3Normal3s.PointNormals.ToPointer(), point3Normal3s.Length);
            Point3Normal3[] pointNormals = span.ToArray();

            return pointNormals;
        }
        #endregion

        #region # 坐标点颜色集重建坐标点颜色数组 —— static Point3Color4[] Recover(this Point3Color4s...
        /// <summary>
        /// 坐标点颜色集重建坐标点颜色数组
        /// </summary>
        public static Point3Color4[] Recover(this Point3Color4s point3Color4s)
        {
            Span<Point3Color4> span = new Span<Point3Color4>(point3Color4s.PointColors.ToPointer(), point3Color4s.Length);
            Point3Color4[] pointColors = span.ToArray();

            return pointColors;
        }
        #endregion

        #region # NARF描述子集重建NARF描述子数组 —— static Narf36F[] Recover(this Narf36Fs narf36Fs)
        /// <summary>
        /// NARF描述子集重建NARF描述子数组
        /// </summary>
        public static Narf36F[] Recover(this Narf36Fs narf36Fs)
        {
            Span<Narf36F> span = new Span<Narf36F>(narf36Fs.Descriptors.ToPointer(), narf36Fs.Length);
            Narf36F[] descriptors = span.ToArray();

            return descriptors;
        }
        #endregion

        #region # PFH描述子集重建PFH描述子数组 —— static PFHSignature125F[] Recover(this PFHSignature125Fs...
        /// <summary>
        /// PFH描述子集重建PFH描述子数组
        /// </summary>
        public static PFHSignature125F[] Recover(this PFHSignature125Fs signature125Fs)
        {
            Span<PFHSignature125F> span = new Span<PFHSignature125F>(signature125Fs.Descriptors.ToPointer(), signature125Fs.Length);
            PFHSignature125F[] descriptors = span.ToArray();

            return descriptors;
        }
        #endregion

        #region # FPFH描述子集重建FPFH描述子数组 —— static FPFHSignature33F[] Recover(this FPFHSignature33Fs...
        /// <summary>
        /// FPFH描述子集重建FPFH描述子数组
        /// </summary>
        public static FPFHSignature33F[] Recover(this FPFHSignature33Fs signature33Fs)
        {
            Span<FPFHSignature33F> span = new Span<FPFHSignature33F>(signature33Fs.Descriptors.ToPointer(), signature33Fs.Length);
            FPFHSignature33F[] descriptors = span.ToArray();

            return descriptors;
        }
        #endregion

        #region # 3DSC描述子集重建3DSC描述子数组 —— static ShapeContext1980F[] Recover(this ShapeContext1980Fs...
        /// <summary>
        /// 3DSC描述子集重建3DSC描述子数组
        /// </summary>
        public static ShapeContext1980F[] Recover(this ShapeContext1980Fs shapeContext1980Fs)
        {
            Span<ShapeContext1980F> span = new Span<ShapeContext1980F>(shapeContext1980Fs.Descriptors.ToPointer(), shapeContext1980Fs.Length);
            ShapeContext1980F[] descriptors = span.ToArray();

            return descriptors;
        }
        #endregion

        #region # SHOT描述子集重建SHOT描述子数组 —— static Shot352F[] Recover(this Shot352Fs shot352Fs)
        /// <summary>
        /// SHOT描述子集重建SHOT描述子数组
        /// </summary>
        public static Shot352F[] Recover(this Shot352Fs shot352Fs)
        {
            Span<Shot352F> span = new Span<Shot352F>(shot352Fs.Descriptors.ToPointer(), shot352Fs.Length);
            Shot352F[] descriptors = span.ToArray();

            return descriptors;
        }
        #endregion
    }
}
