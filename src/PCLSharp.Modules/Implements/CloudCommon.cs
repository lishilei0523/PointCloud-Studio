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
    }
}
