using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Primitives.Models;
using SharpDX;

namespace Sample.Presentation.Maps
{
    /// <summary>
    /// 点映射
    /// </summary>
    public static class PointMap
    {
        /// <summary>
        /// HX坐标点映射坐标点
        /// </summary>
        public static Point3F ToPoint3F(this Geometry3D.Point point)
        {
            return new Point3F(point.P0.X, point.P0.Y, point.P0.Z);
        }

        /// <summary>
        /// 坐标点映射三维向量
        /// </summary>
        public static Vector3 ToVector3(this Point3F point3F)
        {
            return new Vector3(point3F.X, point3F.Y, point3F.Z);
        }

        /// <summary>
        /// 法向量映射三维向量
        /// </summary>
        public static Vector3 ToVector3(this Normal3F normal3F)
        {
            return new Vector3(normal3F.NX, normal3F.NY, normal3F.NZ);
        }

        /// <summary>
        /// 法向量映射线几何构造
        /// </summary>
        /// <param name="normal3F">法向量</param>
        /// <param name="point3F">起始点</param>
        /// <param name="length">长度</param>
        /// <returns>线几何构造</returns>
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
    }
}
