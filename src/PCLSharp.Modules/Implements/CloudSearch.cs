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
    /// 点云搜索实现
    /// </summary>
    public class CloudSearch : ICloudSearch
    {
        #region # K近邻搜索 —— Point3F[] KSearch(IEnumerable<Point3F> points, Point3F referencePoint...
        /// <summary>
        /// K近邻搜索
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="referencePoint">参考坐标点</param>
        /// <param name="k">近邻数量</param>
        /// <returns>结果点集</returns>
        public Point3F[] KSearch(IEnumerable<Point3F> points, Point3F referencePoint, int k)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = SearchNative.KSearch(points_, points_.Length, referencePoint, k);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] resultPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return resultPoints;
        }
        #endregion

        #region # 半径搜索 —— Point3F[] RadiusSearch(IEnumerable<Point3F> points, Point3F referencePoint...
        /// <summary>
        /// 半径搜索
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="referencePoint">参考坐标点</param>
        /// <param name="radius">搜索半径</param>
        /// <returns>结果点集</returns>
        public Point3F[] RadiusSearch(IEnumerable<Point3F> points, Point3F referencePoint, float radius)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = SearchNative.RadiusSearch(points_, points_.Length, referencePoint, radius);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] resultPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return resultPoints;
        }
        #endregion

        #region # 八叉树搜索 —— Point3F[] OctreeSearch(IEnumerable<Point3F> points, Point3F referencePoint...
        /// <summary>
        /// 八叉树搜索
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="referencePoint">参考坐标点</param>
        /// <param name="resolution">分辨率</param>
        /// <returns>结果点集</returns>
        public Point3F[] OctreeSearch(IEnumerable<Point3F> points, Point3F referencePoint, float resolution)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Point3F>();
            }

            #endregion

            IntPtr pointer = SearchNative.OctreeSearch(points_, points_.Length, referencePoint, resolution);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] resultPoints = point3Fs.Recover();
            DisposeNative.DisposePoint3Fs(pointer);

            return resultPoints;
        }
        #endregion
    }
}
