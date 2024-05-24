using PCLSharp.Primitives.Constants;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Declarations
{
    /// <summary>
    /// 释放资源声明
    /// </summary>
    public static class DisposeNative
    {
        #region # 释放坐标点 —— static extern void DisposePoint3F(IntPtr pointer)
        /// <summary>
        /// 释放坐标点
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Primitives, EntryPoint = "disposePoint3F")]
        public static extern void DisposePoint3F(IntPtr pointer);
        #endregion

        #region # 释放坐标点集 —— static extern void DisposePoint3Fs(IntPtr pointer)
        /// <summary>
        /// 释放坐标点集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Primitives, EntryPoint = "disposePoint3Fs")]
        public static extern void DisposePoint3Fs(IntPtr pointer);
        #endregion

        #region # 释放法向量 —— static extern void DisposeNormal3F(IntPtr pointer)
        /// <summary>
        /// 释放法向量
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Primitives, EntryPoint = "disposeNormal3F")]
        public static extern void DisposeNormal3F(IntPtr pointer);
        #endregion

        #region # 释放法向量集 —— static extern void DisposeNormal3Fs(IntPtr pointer)
        /// <summary>
        /// 释放法向量集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Primitives, EntryPoint = "disposeNormal3Fs")]
        public static extern void DisposeNormal3Fs(IntPtr pointer);
        #endregion

        #region # 释放坐标点法向量 —— static extern void DisposePoint3Normal3(IntPtr pointer)
        /// <summary>
        /// 释放坐标点法向量
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Primitives, EntryPoint = "disposePoint3Normal3")]
        public static extern void DisposePoint3Normal3(IntPtr pointer);
        #endregion

        #region # 释放坐标点法向量集 —— static extern void DisposePoint3Normal3s(IntPtr pointer)
        /// <summary>
        /// 释放坐标点法向量集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Primitives, EntryPoint = "disposePoint3Normal3s")]
        public static extern void DisposePoint3Normal3s(IntPtr pointer);
        #endregion

        #region # 释放坐标点颜色 —— static extern void DisposePoint3Color4(IntPtr pointer)
        /// <summary>
        /// 释放坐标点颜色
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Primitives, EntryPoint = "disposePoint3Color4")]
        public static extern void DisposePoint3Color4(IntPtr pointer);
        #endregion

        #region # 释放坐标点颜色集 —— static extern void DisposePoint3Color4s(IntPtr pointer)
        /// <summary>
        /// 释放坐标点颜色集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Primitives, EntryPoint = "disposePoint3Color4s")]
        public static extern void DisposePoint3Color4s(IntPtr pointer);
        #endregion
    }
}
