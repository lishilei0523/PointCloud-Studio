using PCLSharp.FileIO.Declarations;
using PCLSharp.FileIO.Interfaces;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Models;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;

namespace PCLSharp.FileIO.Implements
{
    /// <summary>
    /// 点云读写器实现
    /// </summary>
    public class CloudConductor : ICloudConductor
    {
        #region # 加载PCD文件 —— Point3F[] LoadPCD(string filePath)
        /// <summary>
        /// 加载PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        public Point3F[] LoadPCD(string filePath)
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!File.Exists(filePath))
            {
                throw new ArgumentOutOfRangeException(nameof(filePath), "文件路径不存在！");
            }

            #endregion

            IntPtr pointer = ConductorNative.LoadPCD(filePath);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] points = point3Fs.ToPoint3Fs();
            ConductorNative.Dispose(pointer);

            return points;
        }
        #endregion

        #region # 加载法向量PCD文件 —— Point3Normal3[] LoadNormalPCD(string filePath)
        /// <summary>
        /// 加载法向量PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        public Point3Normal3[] LoadNormalPCD(string filePath)
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!File.Exists(filePath))
            {
                throw new ArgumentOutOfRangeException(nameof(filePath), "文件路径不存在！");
            }

            #endregion

            IntPtr pointer = ConductorNative.LoadNormalPCD(filePath);
            Point3Normal3s point3Normal3s = Marshal.PtrToStructure<Point3Normal3s>(pointer);
            Point3Normal3[] pointNormals = point3Normal3s.ToPoint3Normal3s();
            ConductorNative.Dispose(pointer);

            return pointNormals;
        }
        #endregion

        #region # 加载颜色PCD文件 —— Point3Color4[] LoadColorPCD(string filePath)
        /// <summary>
        /// 加载颜色PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        public Point3Color4[] LoadColorPCD(string filePath)
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!File.Exists(filePath))
            {
                throw new ArgumentOutOfRangeException(nameof(filePath), "文件路径不存在！");
            }

            #endregion

            IntPtr pointer = ConductorNative.LoadColorPCD(filePath);
            Point3Color4s point3Color4s = Marshal.PtrToStructure<Point3Color4s>(pointer);
            Point3Color4[] pointColors = point3Color4s.ToPoint3Color4s();
            ConductorNative.Dispose(pointer);

            return pointColors;
        }
        #endregion

        #region # 加载PLY文件 —— Point3F[] LoadPLY(string filePath)
        /// <summary>
        /// 加载PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        public Point3F[] LoadPLY(string filePath)
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!File.Exists(filePath))
            {
                throw new ArgumentOutOfRangeException(nameof(filePath), "文件路径不存在！");
            }

            #endregion

            IntPtr pointer = ConductorNative.LoadPLY(filePath);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] points = point3Fs.ToPoint3Fs();
            ConductorNative.Dispose(pointer);

            return points;
        }
        #endregion

        #region # 加载法向量PLY文件 —— Point3Normal3[] LoadNormalPLY(string filePath)
        /// <summary>
        /// 加载法向量PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        public Point3Normal3[] LoadNormalPLY(string filePath)
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!File.Exists(filePath))
            {
                throw new ArgumentOutOfRangeException(nameof(filePath), "文件路径不存在！");
            }

            #endregion

            IntPtr pointer = ConductorNative.LoadNormalPLY(filePath);
            Point3Normal3s point3Normal3s = Marshal.PtrToStructure<Point3Normal3s>(pointer);
            Point3Normal3[] pointNormals = point3Normal3s.ToPoint3Normal3s();
            ConductorNative.Dispose(pointer);

            return pointNormals;
        }
        #endregion

        #region # 加载颜色PLY文件 —— Point3Color4[] LoadColorPLY(string filePath)
        /// <summary>
        /// 加载颜色PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        public Point3Color4[] LoadColorPLY(string filePath)
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!File.Exists(filePath))
            {
                throw new ArgumentOutOfRangeException(nameof(filePath), "文件路径不存在！");
            }

            #endregion

            IntPtr pointer = ConductorNative.LoadColorPLY(filePath);
            Point3Color4s point3Color4s = Marshal.PtrToStructure<Point3Color4s>(pointer);
            Point3Color4[] pointColors = point3Color4s.ToPoint3Color4s();
            ConductorNative.Dispose(pointer);

            return pointColors;
        }
        #endregion

        #region # 加载OBJ文件 —— Point3F[] LoadOBJ(string filePath)
        /// <summary>
        /// 加载OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        public Point3F[] LoadOBJ(string filePath)
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!File.Exists(filePath))
            {
                throw new ArgumentOutOfRangeException(nameof(filePath), "文件路径不存在！");
            }

            #endregion

            IntPtr pointer = ConductorNative.LoadOBJ(filePath);
            Point3Fs point3Fs = Marshal.PtrToStructure<Point3Fs>(pointer);
            Point3F[] points = point3Fs.ToPoint3Fs();
            ConductorNative.Dispose(pointer);

            return points;
        }
        #endregion

        #region # 加载法向量OBJ文件 —— Point3Normal3[] LoadNormalOBJ(string filePath)
        /// <summary>
        /// 加载法向量OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        public Point3Normal3[] LoadNormalOBJ(string filePath)
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!File.Exists(filePath))
            {
                throw new ArgumentOutOfRangeException(nameof(filePath), "文件路径不存在！");
            }

            #endregion

            IntPtr pointer = ConductorNative.LoadNormalOBJ(filePath);
            Point3Normal3s point3Normal3s = Marshal.PtrToStructure<Point3Normal3s>(pointer);
            Point3Normal3[] pointNormals = point3Normal3s.ToPoint3Normal3s();
            ConductorNative.Dispose(pointer);

            return pointNormals;
        }
        #endregion

        #region # 加载颜色OBJ文件 —— Point3Color4[] LoadColorOBJ(string filePath)
        /// <summary>
        /// 加载颜色OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        public Point3Color4[] LoadColorOBJ(string filePath)
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!File.Exists(filePath))
            {
                throw new ArgumentOutOfRangeException(nameof(filePath), "文件路径不存在！");
            }

            #endregion

            IntPtr pointer = ConductorNative.LoadColorOBJ(filePath);
            Point3Color4s point3Color4s = Marshal.PtrToStructure<Point3Color4s>(pointer);
            Point3Color4[] pointColors = point3Color4s.ToPoint3Color4s();
            ConductorNative.Dispose(pointer);

            return pointColors;
        }
        #endregion

        #region # 保存PCD文本文件 —— void SaveTextPCD(IEnumerable<Point3F> points, string filePath)
        /// <summary>
        /// 保存PCD文本文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveTextPCD(IEnumerable<Point3F> points, string filePath)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!points_.Any())
            {
                throw new ArgumentNullException(nameof(points), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveTextPCD(points_, points_.Length, filePath);
        }
        #endregion

        #region # 保存PCD二进制文件 —— void SaveBinaryPCD(IEnumerable<Point3F> points, string filePath)
        /// <summary>
        /// 保存PCD二进制文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveBinaryPCD(IEnumerable<Point3F> points, string filePath)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!points_.Any())
            {
                throw new ArgumentNullException(nameof(points), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveBinaryPCD(points_, points_.Length, filePath);
        }
        #endregion

        #region # 保存法向量PCD文本文件 —— void SaveNormalTextPCD(IEnumerable<Point3Normal3> pointNormals...
        /// <summary>
        /// 保存法向量PCD文本文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveNormalTextPCD(IEnumerable<Point3Normal3> pointNormals, string filePath)
        {
            Point3Normal3[] pointNormals_ = pointNormals?.ToArray() ?? Array.Empty<Point3Normal3>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!pointNormals_.Any())
            {
                throw new ArgumentNullException(nameof(pointNormals), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveNormalTextPCD(pointNormals_, pointNormals_.Length, filePath);
        }
        #endregion

        #region # 保存法向量PCD二进制文件 —— void SaveNormalBinaryPCD(IEnumerable<Point3Normal3> pointNormals...
        /// <summary>
        /// 保存法向量PCD二进制文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveNormalBinaryPCD(IEnumerable<Point3Normal3> pointNormals, string filePath)
        {
            Point3Normal3[] pointNormals_ = pointNormals?.ToArray() ?? Array.Empty<Point3Normal3>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!pointNormals_.Any())
            {
                throw new ArgumentNullException(nameof(pointNormals), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveNormalBinaryPCD(pointNormals_, pointNormals_.Length, filePath);
        }
        #endregion

        #region # 保存颜色PCD文本文件 —— void SaveColorTextPCD(IEnumerable<Point3Color4> pointColors...
        /// <summary>
        /// 保存颜色PCD文本文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveColorTextPCD(IEnumerable<Point3Color4> pointColors, string filePath)
        {
            Point3Color4[] pointColors_ = pointColors?.ToArray() ?? Array.Empty<Point3Color4>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!pointColors_.Any())
            {
                throw new ArgumentNullException(nameof(pointColors), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveColorTextPCD(pointColors_, pointColors_.Length, filePath);
        }
        #endregion

        #region # 保存颜色PCD二进制文件 —— void SaveColorBinaryPCD(IEnumerable<Point3Color4> pointColors...
        /// <summary>
        /// 保存颜色PCD二进制文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveColorBinaryPCD(IEnumerable<Point3Color4> pointColors, string filePath)
        {
            Point3Color4[] pointColors_ = pointColors?.ToArray() ?? Array.Empty<Point3Color4>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!pointColors_.Any())
            {
                throw new ArgumentNullException(nameof(pointColors), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveColorBinaryPCD(pointColors_, pointColors_.Length, filePath);
        }
        #endregion

        #region # 保存PLY文本文件 —— void SaveTextPLY(IEnumerable<Point3F> points, string filePath)
        /// <summary>
        /// 保存PLY文本文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveTextPLY(IEnumerable<Point3F> points, string filePath)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!points_.Any())
            {
                throw new ArgumentNullException(nameof(points), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveTextPLY(points_, points_.Length, filePath);
        }
        #endregion

        #region # 保存PLY二进制文件 —— void SaveBinaryPLY(IEnumerable<Point3F> points, string filePath)
        /// <summary>
        /// 保存PLY二进制文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveBinaryPLY(IEnumerable<Point3F> points, string filePath)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!points_.Any())
            {
                throw new ArgumentNullException(nameof(points), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveBinaryPLY(points_, points_.Length, filePath);
        }
        #endregion

        #region # 保存法向量PLY文本文件 —— void SaveNormalTextPLY(IEnumerable<Point3Normal3> pointNormals...
        /// <summary>
        /// 保存法向量PLY文本文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveNormalTextPLY(IEnumerable<Point3Normal3> pointNormals, string filePath)
        {
            Point3Normal3[] pointNormals_ = pointNormals?.ToArray() ?? Array.Empty<Point3Normal3>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!pointNormals_.Any())
            {
                throw new ArgumentNullException(nameof(pointNormals), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveNormalTextPLY(pointNormals_, pointNormals_.Length, filePath);
        }
        #endregion

        #region # 保存法向量PLY二进制文件 —— void SaveNormalBinaryPLY(IEnumerable<Point3Normal3> pointNormals...
        /// <summary>
        /// 保存法向量PLY二进制文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveNormalBinaryPLY(IEnumerable<Point3Normal3> pointNormals, string filePath)
        {
            Point3Normal3[] pointNormals_ = pointNormals?.ToArray() ?? Array.Empty<Point3Normal3>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!pointNormals_.Any())
            {
                throw new ArgumentNullException(nameof(pointNormals), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveNormalBinaryPLY(pointNormals_, pointNormals_.Length, filePath);
        }
        #endregion

        #region # 保存颜色PLY文本文件 —— void SaveColorTextPLY(IEnumerable<Point3Color4> pointColors...
        /// <summary>
        /// 保存颜色PLY文本文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveColorTextPLY(IEnumerable<Point3Color4> pointColors, string filePath)
        {
            Point3Color4[] pointColors_ = pointColors?.ToArray() ?? Array.Empty<Point3Color4>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!pointColors_.Any())
            {
                throw new ArgumentNullException(nameof(pointColors), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveColorTextPLY(pointColors_, pointColors_.Length, filePath);
        }
        #endregion

        #region # 保存颜色PLY二进制文件 —— void SaveColorBinaryPLY(IEnumerable<Point3Color4> pointColors...
        /// <summary>
        /// 保存颜色PLY二进制文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="filePath">文件路径</param>
        public void SaveColorBinaryPLY(IEnumerable<Point3Color4> pointColors, string filePath)
        {
            Point3Color4[] pointColors_ = pointColors?.ToArray() ?? Array.Empty<Point3Color4>();

            #region # 验证

            if (string.IsNullOrWhiteSpace(filePath))
            {
                throw new ArgumentNullException(nameof(filePath), "文件路径不可为空！");
            }
            if (!pointColors_.Any())
            {
                throw new ArgumentNullException(nameof(pointColors), "要保存的点集不可为空！");
            }

            #endregion

            ConductorNative.SaveColorBinaryPLY(pointColors_, pointColors_.Length, filePath);
        }
        #endregion
    }
}
