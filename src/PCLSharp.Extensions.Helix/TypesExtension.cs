using PCLSharp.Primitives.Models;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
#if NET462_OR_GREATER
using Geometry3D = HelixToolkit.Wpf.SharpDX.Geometry3D;
#endif
#if NET6_0_OR_GREATER
using Geometry3D = HelixToolkit.SharpDX.Core.Geometry3D;
#endif

namespace PCLSharp.Extensions.Helix
{
    /// <summary>
    /// 类型扩展
    /// </summary>
    public static class TypesExtension
    {
        #region # WPF坐标点映射坐标点 —— static Point3F ToPoint3F(this Point3D point3D)
        /// <summary>
        /// WPF坐标点映射坐标点
        /// </summary>
        public static Point3F ToPoint3F(this Point3D point3D)
        {
            Point3F point3F = new Point3F((float)point3D.X, (float)point3D.Y, (float)point3D.Z);

            return point3F;
        }
        #endregion

        #region # 坐标点映射WPF坐标点 —— static Point3D ToPoint(this Point3F point3F)
        /// <summary>
        /// 坐标点映射WPF坐标点
        /// </summary>
        public static Point3D ToPoint(this Point3F point3F)
        {
            Point3D point3D = new Point3D(point3F.X, point3F.Y, point3F.Z);

            return point3D;
        }
        #endregion

        #region # Helix坐标点映射坐标点 —— static Point3F ToPoint3F(this Geometry3D.Point point)
        /// <summary>
        /// Helix坐标点映射坐标点
        /// </summary>
        public static Point3F ToPoint3F(this Geometry3D.Point point)
        {
            return new Point3F(point.P0.X, point.P0.Y, point.P0.Z);
        }
        #endregion

        #region # Helix坐标点集映射坐标点集 —— static IEnumerable<Point3F> ToPoint3Fs(this IEnumerable...
        /// <summary>
        /// Helix坐标点集映射坐标点集
        /// </summary>
        public static IEnumerable<Point3F> ToPoint3Fs(this IEnumerable<Geometry3D.Point> points)
        {
            IEnumerable<Point3F> point3Fs = points.Select(point => point.ToPoint3F());

            return point3Fs;
        }
        #endregion

        #region # 坐标点映射Helix坐标点 —— static Geometry3D.Point ToHelixPoint(this Point3F point3F)
        /// <summary>
        /// 坐标点映射Helix坐标点
        /// </summary>
        public static Geometry3D.Point ToHelixPoint(this Point3F point3F)
        {
            Geometry3D.Point helixPoint = new Geometry3D.Point();
            helixPoint.P0.X = point3F.X;
            helixPoint.P0.Y = point3F.Y;
            helixPoint.P0.Z = point3F.Z;

            return helixPoint;
        }
        #endregion

        #region # WPF颜色映射颜色 —— static Color4F ToColor4F(this Color color)
        /// <summary>
        /// WPF颜色映射颜色
        /// </summary>
        public static Color4F ToColor4F(this Color color)
        {
            Color4F color4F = new Color4F(color.R, color.G, color.B, color.A);

            return color4F;
        }
        #endregion

        #region # 颜色映射WPF颜色 —— static Color ToColor(this Color4F color4F)
        /// <summary>
        /// 颜色映射WPF颜色
        /// </summary>
        public static Color ToColor(this Color4F color4F)
        {
            Color color = new Color(color4F.R, color4F.G, color4F.B, color4F.A);

            return color;
        }
        #endregion

        #region # Helix颜色映射颜色 —— static Color4F ToColor4F(this Color4 color4)
        /// <summary>
        /// Helix颜色映射颜色
        /// </summary>
        public static Color4F ToColor4F(this Color4 color4)
        {
            byte r = Convert.ToByte(color4.Red * 255.0f);
            byte g = Convert.ToByte(color4.Green * 255.0f);
            byte b = Convert.ToByte(color4.Blue * 255.0f);
            byte a = Convert.ToByte(color4.Alpha * 255.0f);
            Color4F color4F = new Color4F(r, g, b, a);

            return color4F;
        }
        #endregion

        #region # 颜色映射Helix颜色 —— static Color4 ToColor4(this Color4F color4F)
        /// <summary>
        /// 颜色映射Helix颜色
        /// </summary>
        public static Color4 ToColor4(this Color4F color4F)
        {
            float r = color4F.R / 255.0f;
            float g = color4F.G / 255.0f;
            float b = color4F.B / 255.0f;
            float a = color4F.A / 255.0f;
            Color4 color4 = new Color4(r, g, b, a);

            return color4;
        }
        #endregion

        #region # Helix三维向量映射坐标点 —— static Point3F ToPoint3F(this Vector3 vector3)
        /// <summary>
        /// Helix三维向量映射坐标点
        /// </summary>
        public static Point3F ToPoint3F(this Vector3 vector3)
        {
            return new Point3F(vector3.X, vector3.Y, vector3.Z);
        }
        #endregion

        #region # 坐标点映射Helix三维向量 —— static Vector3 ToVector3(this Point3F point3F)
        /// <summary>
        /// 坐标点映射Helix三维向量
        /// </summary>
        public static Vector3 ToVector3(this Point3F point3F)
        {
            return new Vector3(point3F.X, point3F.Y, point3F.Z);
        }
        #endregion

        #region # 坐标点集映射三维向量集 —— static IEnumerable<Vector3> ToVector3s(this IEnumerable...
        /// <summary>
        /// 坐标点集映射三维向量集
        /// </summary>
        public static IEnumerable<Vector3> ToVector3s(this IEnumerable<Point3F> point3Fs)
        {
            IEnumerable<Vector3> vector3s = point3Fs.Select(point => point.ToVector3());

            return vector3s;
        }
        #endregion

        #region # 法向量映射三维向量 —— static Vector3 ToVector3(this Normal3F normal3F)
        /// <summary>
        /// 法向量映射三维向量
        /// </summary>
        public static Vector3 ToVector3(this Normal3F normal3F)
        {
            return new Vector3(normal3F.NX, normal3F.NY, normal3F.NZ);
        }
        #endregion
    }
}
