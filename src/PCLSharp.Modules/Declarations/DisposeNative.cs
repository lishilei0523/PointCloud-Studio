using PCLSharp.Primitives.Constants;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云释放声明
    /// </summary>
    internal static class DisposeNative
    {
        #region # 释放坐标点 —— static extern void DisposePoint3F(IntPtr pointer)
        /// <summary>
        /// 释放坐标点
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePoint3F")]
        public static extern void DisposePoint3F(IntPtr pointer);
        #endregion

        #region # 释放坐标点集 —— static extern void DisposePoint3Fs(IntPtr pointer)
        /// <summary>
        /// 释放坐标点集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePoint3Fs")]
        public static extern void DisposePoint3Fs(IntPtr pointer);
        #endregion

        #region # 释放坐标点集分组 —— static extern void DisposePoint3FsGroup(IntPtr pointer...
        /// <summary>
        /// 释放坐标点集分组
        /// </summary>
        /// <param name="pointer">二级指针</param>
        /// <param name="groupCount">分组数</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePoint3FsGroup")]
        public static extern void DisposePoint3FsGroup(IntPtr pointer, int groupCount);
        #endregion

        #region # 释放法向量 —— static extern void DisposeNormal3F(IntPtr pointer)
        /// <summary>
        /// 释放法向量
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeNormal3F")]
        public static extern void DisposeNormal3F(IntPtr pointer);
        #endregion

        #region # 释放法向量集 —— static extern void DisposeNormal3Fs(IntPtr pointer)
        /// <summary>
        /// 释放法向量集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeNormal3Fs")]
        public static extern void DisposeNormal3Fs(IntPtr pointer);
        #endregion

        #region # 释放法向量集分组 —— static extern void DisposeNormal3FsGroup(IntPtr pointer...
        /// <summary>
        /// 释放法向量集分组
        /// </summary>
        /// <param name="pointer">二级指针</param>
        /// <param name="groupCount">分组数</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeNormal3FsGroup")]
        public static extern void DisposeNormal3FsGroup(IntPtr pointer, int groupCount);
        #endregion

        #region # 释放坐标点法向量 —— static extern void DisposePoint3Normal3(IntPtr pointer)
        /// <summary>
        /// 释放坐标点法向量
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePoint3Normal3")]
        public static extern void DisposePoint3Normal3(IntPtr pointer);
        #endregion

        #region # 释放坐标点法向量集 —— static extern void DisposePoint3Normal3s(IntPtr pointer)
        /// <summary>
        /// 释放坐标点法向量集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePoint3Normal3s")]
        public static extern void DisposePoint3Normal3s(IntPtr pointer);
        #endregion

        #region # 释放坐标点法向量集分组 —— static extern void DisposePoint3Normal3sGroup(IntPtr pointer...
        /// <summary>
        /// 释放坐标点法向量集分组
        /// </summary>
        /// <param name="pointer">二级指针</param>
        /// <param name="groupCount">分组数</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePoint3Normal3sGroup")]
        public static extern void DisposePoint3Normal3sGroup(IntPtr pointer, int groupCount);
        #endregion

        #region # 释放坐标点颜色 —— static extern void DisposePoint3Color4(IntPtr pointer)
        /// <summary>
        /// 释放坐标点颜色
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePoint3Color4")]
        public static extern void DisposePoint3Color4(IntPtr pointer);
        #endregion

        #region # 释放坐标点颜色集 —— static extern void DisposePoint3Color4s(IntPtr pointer)
        /// <summary>
        /// 释放坐标点颜色集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePoint3Color4s")]
        public static extern void DisposePoint3Color4s(IntPtr pointer);
        #endregion

        #region # 释放坐标点颜色集分组 —— static extern void DisposePoint3Color4sGroup(IntPtr pointer...
        /// <summary>
        /// 释放坐标点颜色集分组
        /// </summary>
        /// <param name="pointer">二级指针</param>
        /// <param name="groupCount">分组数</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePoint3Color4sGroup")]
        public static extern void DisposePoint3Color4sGroup(IntPtr pointer, int groupCount);
        #endregion

        #region # 释放NARF描述子 —— static extern void DisposeNarf36F(IntPtr pointer)
        /// <summary>
        /// 释放NARF描述子
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeNarf36F")]
        public static extern void DisposeNarf36F(IntPtr pointer);
        #endregion

        #region # 释放NARF描述子集 —— static extern void DisposeNarf36Fs(IntPtr pointer)
        /// <summary>
        /// 释放NARF描述子集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeNarf36Fs")]
        public static extern void DisposeNarf36Fs(IntPtr pointer);
        #endregion

        #region # 释放PFH描述子 —— static extern void DisposePFHSignature125F(IntPtr pointer)
        /// <summary>
        /// 释放PFH描述子
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePFHSignature125F")]
        public static extern void DisposePFHSignature125F(IntPtr pointer);
        #endregion

        #region # 释放PFH描述子集 —— static extern void DisposePFHSignature125Fs(IntPtr pointer)
        /// <summary>
        /// 释放PFH描述子集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposePFHSignature125Fs")]
        public static extern void DisposePFHSignature125Fs(IntPtr pointer);
        #endregion

        #region # 释放FPFH描述子 —— static extern void DisposeFPFHSignature33F(IntPtr pointer)
        /// <summary>
        /// 释放FPFH描述子
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeFPFHSignature33F")]
        public static extern void DisposeFPFHSignature33F(IntPtr pointer);
        #endregion

        #region # 释放FPFH描述子集 —— static extern void DisposeFPFHSignature33Fs(IntPtr pointer)
        /// <summary>
        /// 释放FPFH描述子集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeFPFHSignature33Fs")]
        public static extern void DisposeFPFHSignature33Fs(IntPtr pointer);
        #endregion

        #region # 释放3DSC描述子 —— static extern void DisposeShapeContext1980F(IntPtr pointer)
        /// <summary>
        /// 释放3DSC描述子
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeShapeContext1980F")]
        public static extern void DisposeShapeContext1980F(IntPtr pointer);
        #endregion

        #region # 释放3DSC描述子集 —— static extern void DisposeShapeContext1980Fs(IntPtr pointer)
        /// <summary>
        /// 释放3DSC描述子集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeShapeContext1980Fs")]
        public static extern void DisposeShapeContext1980Fs(IntPtr pointer);
        #endregion

        #region # 释放SHOT描述子 —— static extern void DisposeShot352F(IntPtr pointer)
        /// <summary>
        /// 释放SHOT描述子
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeShot352F")]
        public static extern void DisposeShot352F(IntPtr pointer);
        #endregion

        #region # 释放SHOT描述子集 —— static extern void DisposeShot352Fs(IntPtr pointer)
        /// <summary>
        /// 释放SHOT描述子集
        /// </summary>
        /// <param name="pointer">指针</param>
        [DllImport(AssemblyNames.Modules, EntryPoint = "disposeShot352Fs")]
        public static extern void DisposeShot352Fs(IntPtr pointer);
        #endregion
    }
}
