using PCLSharp.Primitives.Features;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

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
            Narf36F[] descriptors = new Narf36F[narf36Fs.Length];
            int* index = (int*)narf36Fs.Descriptors.ToPointer();
            for (int offset = 0; offset < narf36Fs.Length; offset++)
            {
                IntPtr pointer = new IntPtr(index + offset);
                Narf36F narf36F = Marshal.PtrToStructure<Narf36F>(pointer);
                descriptors[offset] = narf36F;
            }

            return descriptors;
        }
        #endregion

        #region # PFH描述子集重建PFH描述子数组 —— static PFHSignature125F[] Recover(this PFHSignature125Fs...
        /// <summary>
        /// PFH描述子集重建PFH描述子数组
        /// </summary>
        public static PFHSignature125F[] Recover(this PFHSignature125Fs signature125Fs)
        {
            PFHSignature125F[] descriptors = new PFHSignature125F[signature125Fs.Length];
            int* index = (int*)signature125Fs.Descriptors.ToPointer();
            for (int offset = 0; offset < signature125Fs.Length; offset++)
            {
                IntPtr pointer = new IntPtr(index + offset);
                PFHSignature125F signature125F = Marshal.PtrToStructure<PFHSignature125F>(pointer);
                descriptors[offset] = signature125F;
            }

            return descriptors;
        }
        #endregion

        #region # FPFH描述子集重建FPFH描述子数组 —— static FPFHSignature33F[] Recover(this FPFHSignature33Fs...
        /// <summary>
        /// FPFH描述子集重建FPFH描述子数组
        /// </summary>
        public static FPFHSignature33F[] Recover(this FPFHSignature33Fs signature33Fs)
        {
            FPFHSignature33F[] descriptors = new FPFHSignature33F[signature33Fs.Length];
            int* index = (int*)signature33Fs.Descriptors.ToPointer();
            for (int offset = 0; offset < signature33Fs.Length; offset++)
            {
                IntPtr pointer = new IntPtr(index + offset);
                FPFHSignature33F signature33F = Marshal.PtrToStructure<FPFHSignature33F>(pointer);
                descriptors[offset] = signature33F;
            }

            return descriptors;
        }
        #endregion

        #region # 3DSC描述子集重建3DSC描述子数组 —— static ShapeContext1980F[] Recover(this ShapeContext1980Fs...
        /// <summary>
        /// 3DSC描述子集重建3DSC描述子数组
        /// </summary>
        public static ShapeContext1980F[] Recover(this ShapeContext1980Fs shapeContext1980Fs)
        {
            ShapeContext1980F[] descriptors = new ShapeContext1980F[shapeContext1980Fs.Length];
            int* index = (int*)shapeContext1980Fs.Descriptors.ToPointer();
            for (int offset = 0; offset < shapeContext1980Fs.Length; offset++)
            {
                IntPtr pointer = new IntPtr(index + offset);
                ShapeContext1980F shapeContext1980F = Marshal.PtrToStructure<ShapeContext1980F>(pointer);
                descriptors[offset] = shapeContext1980F;
            }

            return descriptors;
        }
        #endregion

        #region # SHOT描述子集重建SHOT描述子数组 —— static Shot352F[] Recover(this Shot352Fs shot352Fs)
        /// <summary>
        /// SHOT描述子集重建SHOT描述子数组
        /// </summary>
        public static Shot352F[] Recover(this Shot352Fs shot352Fs)
        {
            Shot352F[] descriptors = new Shot352F[shot352Fs.Length];
            int* index = (int*)shot352Fs.Descriptors.ToPointer();
            for (int offset = 0; offset < shot352Fs.Length; offset++)
            {
                IntPtr pointer = new IntPtr(index + offset);
                Shot352F shot352F = Marshal.PtrToStructure<Shot352F>(pointer);
                descriptors[offset] = shot352F;
            }

            return descriptors;
        }
        #endregion
    }
}
