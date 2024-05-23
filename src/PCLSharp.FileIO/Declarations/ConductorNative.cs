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

        #region # 加载法向量PCD文件 —— static extern IntPtr LoadNormalPCD(string filePath)
        /// <summary>
        /// 加载法向量PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "loadNormalPCD")]
        public static extern IntPtr LoadNormalPCD(string filePath);
        #endregion

        #region # 加载颜色PCD文件 —— static extern IntPtr LoadColorPCD(string filePath)
        /// <summary>
        /// 加载颜色PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "loadColorPCD")]
        public static extern IntPtr LoadColorPCD(string filePath);
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

        #region # 加载法向量PLY文件 —— static extern IntPtr LoadNormalPLY(string filePath)
        /// <summary>
        /// 加载法向量PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "loadNormalPLY")]
        public static extern IntPtr LoadNormalPLY(string filePath);
        #endregion

        #region # 加载颜色PLY文件 —— static extern IntPtr LoadColorPLY(string filePath)
        /// <summary>
        /// 加载颜色PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "loadColorPLY")]
        public static extern IntPtr LoadColorPLY(string filePath);
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

        #region # 加载法向量OBJ文件 —— static extern IntPtr LoadNormalOBJ(string filePath)
        /// <summary>
        /// 加载法向量OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "loadNormalOBJ")]
        public static extern IntPtr LoadNormalOBJ(string filePath);
        #endregion

        #region # 加载颜色OBJ文件 —— static extern IntPtr LoadColorOBJ(string filePath)
        /// <summary>
        /// 加载颜色OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "loadColorOBJ")]
        public static extern IntPtr LoadColorOBJ(string filePath);
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

        #region # 保存法向量PCD文本文件 —— static extern void SaveNormalTextPCD(Point3Normal3[] pointNormals...
        /// <summary>
        /// 保存法向量PCD文本文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveNormalTextPCD")]
        public static extern void SaveNormalTextPCD(Point3Normal3[] pointNormals, int length, string filePath);
        #endregion

        #region # 保存法向量PCD二进制文件 —— static extern void SaveNormalBinaryPCD(Point3Normal3[] pointNormals...
        /// <summary>
        /// 保存法向量PCD二进制文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveNormalBinaryPCD")]
        public static extern void SaveNormalBinaryPCD(Point3Normal3[] pointNormals, int length, string filePath);
        #endregion

        #region # 保存颜色PCD文本文件 —— static extern void SaveColorTextPCD(Point3Color4[] pointColors...
        /// <summary>
        /// 保存颜色PCD文本文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveColorTextPCD")]
        public static extern void SaveColorTextPCD(Point3Color4[] pointColors, int length, string filePath);
        #endregion

        #region # 保存颜色PCD二进制文件 —— static extern void SaveColorBinaryPCD(Point3Color4[] pointColors...
        /// <summary>
        /// 保存颜色PCD二进制文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveColorBinaryPCD")]
        public static extern void SaveColorBinaryPCD(Point3Color4[] pointColors, int length, string filePath);
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

        #region # 保存法向量PLY文本文件 —— static extern void SaveNormalTextPLY(Point3Normal3[] pointNormals...
        /// <summary>
        /// 保存法向量PLY文本文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveNormalTextPLY")]
        public static extern void SaveNormalTextPLY(Point3Normal3[] pointNormals, int length, string filePath);
        #endregion

        #region # 保存法向量PLY二进制文件 —— static extern void SaveNormalBinaryPLY(Point3Normal3[] pointNormals...
        /// <summary>
        /// 保存法向量PLY二进制文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveNormalBinaryPLY")]
        public static extern void SaveNormalBinaryPLY(Point3Normal3[] pointNormals, int length, string filePath);
        #endregion

        #region # 保存颜色PLY文本文件 —— static extern void SaveColorTextPLY(Point3Color4[] pointColors...
        /// <summary>
        /// 保存颜色PLY文本文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveColorTextPLY")]
        public static extern void SaveColorTextPLY(Point3Color4[] pointColors, int length, string filePath);
        #endregion

        #region # 保存颜色PLY二进制文件 —— static extern void SaveColorBinaryPLY(Point3Color4[] pointColors...
        /// <summary>
        /// 保存颜色PLY二进制文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="length">点集长度</param>
        /// <param name="filePath">文件路径</param>
        [DllImport(AssemblyNames.FileIO, EntryPoint = "saveColorBinaryPLY")]
        public static extern void SaveColorBinaryPLY(Point3Color4[] pointColors, int length, string filePath);
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
