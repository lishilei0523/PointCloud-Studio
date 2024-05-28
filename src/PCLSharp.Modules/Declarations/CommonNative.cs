using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云通用操作声明
    /// </summary>
    public static class CommonNative
    {
        #region # 估算质心 —— static extern IntPtr EstimateCentroid(Point3F[] points...
        /// <summary>
        /// 估算质心
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <returns>质心坐标点</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "estimateCentroid")]
        public static extern IntPtr EstimateCentroid(Point3F[] points, int length);
        #endregion

        #region # 仿射变换 —— static extern IntPtr AffineTransform(Point3F[] points...
        /// <summary>
        /// 仿射变换
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="pose">位姿</param>
        /// <returns>变换后点云</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "affineTransform")]
        public static extern IntPtr AffineTransform(Point3F[] points, int length, Pose pose);
        #endregion
    }
}
