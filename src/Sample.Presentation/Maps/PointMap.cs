using HelixToolkit.Wpf.SharpDX;
using Sample.Presentation.Models;
using SharpDX;

namespace Sample.Presentation.Maps
{
    /// <summary>
    /// 点映射
    /// </summary>
    public static class PointMap
    {
        /// <summary>
        /// HX坐标点映射3D坐标点
        /// </summary>
        public static Point3F ToPoint3F(this Geometry3D.Point point)
        {
            return new Point3F(point.P0.X, point.P0.Y, point.P0.Z);
        }

        /// <summary>
        /// 3D坐标点映射三维向量
        /// </summary>
        public static Vector3 ToVector3(this Point3F point3F)
        {
            return new Vector3(point3F.X, point3F.Y, point3F.Z);
        }
    }
}
