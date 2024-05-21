using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.FileIO.Declarations
{
    /// <summary>
    /// 点云读写器声明
    /// </summary>
    public static class ConductorNative
    {
        #region # 加载PCD文件 —— static extern IntPtr LoadPCD(string filePath)
        /// <summary>
        /// 加载PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "loadPCD")]
        public static extern IntPtr LoadPCD(string filePath);
        #endregion

        #region # 加载PLY文件 —— static extern IntPtr LoadPLY(string filePath)
        /// <summary>
        /// 加载PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "loadPLY")]
        public static extern IntPtr LoadPLY(string filePath);
        #endregion

        #region # 加载OBJ文件 —— static extern IntPtr LoadOBJ(string filePath)
        /// <summary>
        /// 加载OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "loadOBJ")]
        public static extern IntPtr LoadOBJ(string filePath);
        #endregion

        #region # 保存PCD文本文件 —— static extern void SaveTextPCD(Point3F[] points...
        /// <summary>
        /// 保存PCD文本文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveTextPCD")]
        public static extern void SaveTextPCD(Point3F[] points, int length, string filePath);
        #endregion

        #region # 保存PCD二进制文件 —— static extern void SaveBinaryPCD(Point3F[] points...
        /// <summary>
        /// 保存PCD二进制文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveBinaryPCD")]
        public static extern void SaveBinaryPCD(Point3F[] points, int length, string filePath);
        #endregion

        #region # 保存PLY文本文件 —— static extern void SaveTextPLY(Point3F[] points...
        /// <summary>
        /// 保存PLY文本文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveTextPLY")]
        public static extern void SaveTextPLY(Point3F[] points, int length, string filePath);
        #endregion

        #region # 保存PLY二进制文件 —— static extern void SaveBinaryPLY(Point3F[] points...
        /// <summary>
        /// 保存PLY二进制文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveBinaryPLY")]
        public static extern void SaveBinaryPLY(Point3F[] points, int length, string filePath);
        #endregion

        #region # 释放资源 —— static extern void Dispose(IntPtr pointer)
        /// <summary>
        /// 释放资源
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "dispose")]
        public static extern void Dispose(IntPtr pointer);
        #endregion
    }
}
