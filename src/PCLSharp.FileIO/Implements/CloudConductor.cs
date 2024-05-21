using PCLSharp.FileIO.Declarations;
using PCLSharp.FileIO.Interfaces;
using PCLSharp.Primitives.Maps;
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

        #region # 保存PCD文本文件 —— void SaveTextPCD(string filePath, IEnumerable<Point3F> points)
        /// <summary>
        /// 保存PCD文本文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <param name="points">点集</param>
        public void SaveTextPCD(string filePath, IEnumerable<Point3F> points)
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

        #region # 保存PCD二进制文件 —— void SaveBinaryPCD(string filePath, IEnumerable<Point3F> points)
        /// <summary>
        /// 保存PCD二进制文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <param name="points">点集</param>
        public void SaveBinaryPCD(string filePath, IEnumerable<Point3F> points)
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

        #region # 保存PLY文本文件 —— void SaveTextPLY(string filePath, IEnumerable<Point3F> points)
        /// <summary>
        /// 保存PLY文本文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <param name="points">点集</param>
        public void SaveTextPLY(string filePath, IEnumerable<Point3F> points)
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

        #region # 保存PLY二进制文件 —— void SaveBinaryPLY(string filePath, IEnumerable<Point3F> points)
        /// <summary>
        /// 保存PLY二进制文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <param name="points">点集</param>
        public void SaveBinaryPLY(string filePath, IEnumerable<Point3F> points)
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
    }
}
