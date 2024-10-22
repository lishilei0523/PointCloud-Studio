﻿using PCLSharp.Modules.Declarations;
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
    /// 点云法向量实现
    /// </summary>
    public class CloudNormals : ICloudNormals
    {
        #region # 估算法向量 —— Normal3F[] EstimateNormalsByK(IEnumerable<Point3F> points, int k)
        /// <summary>
        /// 估算法向量
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="k">搜索近邻数量</param>
        /// <returns>法向量集</returns>
        public Normal3F[] EstimateNormalsByK(IEnumerable<Point3F> points, int k)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Normal3F>();
            }

            #endregion

            IntPtr pointer = NormalsNative.EstimateNormalsByK(points_, points_.Length, k);
            Normal3Fs normal3Fs = Marshal.PtrToStructure<Normal3Fs>(pointer);
            Normal3F[] normals = normal3Fs.Recover();
            DisposeNative.DisposeNormal3Fs(pointer);

            return normals;
        }
        #endregion

        #region # 估算法向量 —— Normal3F[] EstimateNormalsByRadius(IEnumerable<Point3F> points, float radius)
        /// <summary>
        /// 估算法向量
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="radius">搜索半径</param>
        /// <returns>法向量集</returns>
        public Normal3F[] EstimateNormalsByRadius(IEnumerable<Point3F> points, float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Normal3F>();
            }

            #endregion

            IntPtr pointer = NormalsNative.EstimateNormalsByRadius(points_, points_.Length, radius);
            Normal3Fs normal3Fs = Marshal.PtrToStructure<Normal3Fs>(pointer);
            Normal3F[] normals = normal3Fs.Recover();
            DisposeNative.DisposeNormal3Fs(pointer);

            return normals;
        }
        #endregion

        #region # 估算法向量 (OMP) —— Normal3F[] EstimateNormalsByKP(IEnumerable<Point3F> points, int k)
        /// <summary>
        /// 估算法向量 (OMP)
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="k">搜索近邻数量</param>
        /// <returns>法向量集</returns>
        public Normal3F[] EstimateNormalsByKP(IEnumerable<Point3F> points, int k)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Normal3F>();
            }

            #endregion

            IntPtr pointer = NormalsNative.EstimateNormalsByKP(points_, points_.Length, k);
            Normal3Fs normal3Fs = Marshal.PtrToStructure<Normal3Fs>(pointer);
            Normal3F[] normals = normal3Fs.Recover();
            DisposeNative.DisposeNormal3Fs(pointer);

            return normals;
        }
        #endregion

        #region # 估算法向量 (OMP) —— Normal3F[] EstimateNormalsByRadiusP(IEnumerable<Point3F> points, float radius)
        /// <summary>
        /// 估算法向量 (OMP)
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="radius">搜索半径</param>
        /// <returns>法向量集</returns>
        public Normal3F[] EstimateNormalsByRadiusP(IEnumerable<Point3F> points, float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Normal3F>();
            }

            #endregion

            IntPtr pointer = NormalsNative.EstimateNormalsByRadiusP(points_, points_.Length, radius);
            Normal3Fs normal3Fs = Marshal.PtrToStructure<Normal3Fs>(pointer);
            Normal3F[] normals = normal3Fs.Recover();
            DisposeNative.DisposeNormal3Fs(pointer);

            return normals;
        }
        #endregion
    }
}
