using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Models;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Linq;
#if NET462_OR_GREATER
using HelixToolkit.Wpf.SharpDX;
#endif
#if NET6_0_OR_GREATER
using HelixToolkit.SharpDX.Core;
using HelixToolkit.Wpf.SharpDX;
#endif

namespace PCLSharp.Extensions.Helix
{
    /// <summary>
    /// 几何扩展
    /// </summary>
    public static class GeometryExtension
    {
        #region # 坐标点集映射点几何 —— static PointGeometry3D ToPointGeometry3D(this IEnumerable<Point3F> points)
        /// <summary>
        /// 坐标点集映射点几何
        /// </summary>
        /// <param name="points">坐标点集</param>
        /// <returns>点几何</returns>
        public static PointGeometry3D ToPointGeometry3D(this IEnumerable<Point3F> points)
        {
            #region # 验证

            if (points == null)
            {
                throw new ArgumentNullException(nameof(points), "坐标点集不可为空！");
            }

            #endregion

            IEnumerable<Vector3> positions = points.ToVector3s();
            PointGeometry3D pointGeometry3D = new PointGeometry3D
            {
                Positions = new Vector3Collection(positions)
            };

            return pointGeometry3D;
        }
        #endregion

        #region # 颜色坐标点集映射点几何 —— static PointGeometry3D ToPointGeometry3D(this IEnumerable<Point3Color4>...
        /// <summary>
        /// 颜色坐标点集映射点几何
        /// </summary>
        /// <param name="pointColors">坐标点集</param>
        /// <returns>点几何</returns>
        public static PointGeometry3D ToPointGeometry3D(this IEnumerable<Point3Color4> pointColors)
        {
            #region # 验证

            pointColors = pointColors?.ToArray() ?? Array.Empty<Point3Color4>();
            if (pointColors == null)
            {
                throw new ArgumentNullException(nameof(pointColors), "颜色坐标点集不可为空！");
            }

            #endregion

            IEnumerable<Vector3> positions = pointColors.Select(x => x.GetPoint().ToVector3());
            IEnumerable<Color4> colors = pointColors.Select(x => x.GetColor().ToColor4());
            PointGeometry3D pointGeometry3D = new PointGeometry3D
            {
                Positions = new Vector3Collection(positions),
                Colors = new Color4Collection(colors)
            };

            return pointGeometry3D;
        }
        #endregion

        #region # 法向量映射线几何 —— static LineGeometry3D ToLineGeometry3D(this Normal3F normal3F, Point3F point3F...
        /// <summary>
        /// 法向量映射线几何
        /// </summary>
        /// <param name="normal3F">法向量</param>
        /// <param name="point3F">起始点</param>
        /// <param name="length">长度</param>
        /// <returns>线几何</returns>
        public static LineGeometry3D ToLineGeometry3D(this Normal3F normal3F, Point3F point3F, float length)
        {
            LineBuilder lineBuilder = new LineBuilder();
            Vector3 origin = point3F.ToVector3();
            Vector3 normal = normal3F.ToVector3();
            Vector3 end = Vector3.Multiply(normal, length) + origin;
            lineBuilder.AddLine(origin, end);
            LineGeometry3D lineGeometry3D = lineBuilder.ToLineGeometry3D();

            return lineGeometry3D;
        }
        #endregion

        #region # 法向量坐标点映射线几何 —— static LineGeometry3D ToLineGeometry3D(this Point3Normal3 pointNormal...
        /// <summary>
        /// 法向量坐标点映射线几何
        /// </summary>
        /// <param name="pointNormal">法向量坐标点</param>
        /// <param name="length">长度</param>
        /// <returns>线几何</returns>
        public static LineGeometry3D ToLineGeometry3D(this Point3Normal3 pointNormal, float length)
        {
            Normal3F normal = pointNormal.GetNormal();
            Point3F origin = pointNormal.GetPoint();
            LineGeometry3D lineGeometry3D = normal.ToLineGeometry3D(origin, length);

            return lineGeometry3D;
        }
        #endregion
    }
}
