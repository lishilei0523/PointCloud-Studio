using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云搜索声明
    /// </summary>
    public static class SearchNative
    {
        #region # K近邻搜索 —— static extern IntPtr KSearch(Point3F[] points...
        /// <summary>
        /// K近邻搜索
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="referencePoint">参考坐标点</param>
        /// <param name="k">近邻数量</param>
        /// <returns>结果点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "kSearch")]
        public static extern IntPtr KSearch(Point3F[] points, int length, Point3F referencePoint, int k);
        #endregion

        #region # 半径搜索 —— static extern IntPtr RadiusSearch(Point3F[] points...
        /// <summary>
        /// 半径搜索
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="referencePoint">参考坐标点</param>
        /// <param name="radius">搜索半径</param>
        /// <returns>结果点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "radiusSearch")]
        public static extern IntPtr RadiusSearch(Point3F[] points, int length, Point3F referencePoint, float radius);
        #endregion

        #region # 八叉树搜索 —— static extern IntPtr OctreeSearch(Point3F[] points...
        /// <summary>
        /// 八叉树搜索
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="referencePoint">参考坐标点</param>
        /// <param name="resolution">分辨率</param>
        /// <returns>结果点集</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "octreeSearch")]
        public static extern IntPtr OctreeSearch(Point3F[] points, int length, Point3F referencePoint, float resolution);
        #endregion
    }
}
