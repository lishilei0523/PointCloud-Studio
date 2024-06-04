using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Features;
using PCLSharp.Primitives.Models;
using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Declarations
{
    /// <summary>
    /// 点云配准声明
    /// </summary>
    internal static class RegistrationsNative
    {
        #region # 4PCS配准 —— static extern IntPtr Align4PCS(Point3F[] sourcePoints...
        /// <summary>
        /// 4PCS配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="approxOverlap">近似重叠</param>
        /// <param name="delta">配准距离</param>
        /// <param name="normalize">标准化</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="maxComputationTime">最大计算时间(秒)</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "align4PCS")]
        public static extern IntPtr Align4PCS(Point3F[] sourcePoints, int sourceLength, Point3F[] targetPoints, int targetLength, float approxOverlap, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);
        #endregion

        #region # K-4PCS配准 —— static extern IntPtr AlignK4PCS(Point3F[] sourcePoints...
        /// <summary>
        /// K-4PCS配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="approxOverlap">近似重叠</param>
        /// <param name="lambda">平移向量加权系数</param>
        /// <param name="delta">配准距离</param>
        /// <param name="normalize">标准化</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="maxComputationTime">最大计算时间(秒)</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "alignK4PCS")]
        public static extern IntPtr AlignK4PCS(Point3F[] sourcePoints, int sourceLength, Point3F[] targetPoints, int targetLength, float approxOverlap, float lambda, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);
        #endregion

        #region # NDT配准 —— static extern IntPtr AlignNDT(Point3F[] sourcePoints...
        /// <summary>
        /// NDT配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="resolution">分辨率</param>
        /// <param name="stepSize">步长</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "alignNDT")]
        public static extern IntPtr AlignNDT(Point3F[] sourcePoints, int sourceLength, Point3F[] targetPoints, int targetLength, float resolution, float stepSize, float transformationEpsilon, int maximumIterations);
        #endregion

        #region # GICP配准 —— static extern IntPtr AlignGICP(Point3F[] sourcePoints...
        /// <summary>
        /// GICP配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="maxCorrespondenceDistance">分辨率</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "alignGICP")]
        public static extern IntPtr AlignGICP(Point3F[] sourcePoints, int sourceLength, Point3F[] targetPoints, int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);
        #endregion

        #region # SAC-IA-NARF配准 —— static extern IntPtr SaciaAlignNARF(Point3F[] sourcePoints...
        /// <summary>
        /// SAC-IA-NARF配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源NARF描述子集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标NARF描述子集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "saciaAlignNARF")]
        public static extern IntPtr SaciaAlignNARF(Point3F[] sourcePoints, Narf36F[] sourceDescriptors, int sourceLength, Point3F[] targetPoints, Narf36F[] targetDescriptors, int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);
        #endregion

        #region # SAC-IA-PFH配准 —— static extern IntPtr SaciaAlignPFH(Point3F[] sourcePoints...
        /// <summary>
        /// SAC-IA-PFH配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源PFH描述子集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标PFH描述子集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "saciaAlignPFH")]
        public static extern IntPtr SaciaAlignPFH(Point3F[] sourcePoints, PFHSignature125F[] sourceDescriptors, int sourceLength, Point3F[] targetPoints, PFHSignature125F[] targetDescriptors, int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);
        #endregion

        #region # SAC-IA-FPFH配准 —— static extern IntPtr SaciaAlignFPFH(Point3F[] sourcePoints...
        /// <summary>
        /// SAC-IA-FPFH配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源FPFH描述子集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标FPFH描述子集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "saciaAlignFPFH")]
        public static extern IntPtr SaciaAlignFPFH(Point3F[] sourcePoints, FPFHSignature33F[] sourceDescriptors, int sourceLength, Point3F[] targetPoints, FPFHSignature33F[] targetDescriptors, int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);
        #endregion

        #region # SAC-IA-3DSC配准 —— static extern IntPtr SaciaAlign3DSC(Point3F[] sourcePoints...
        /// <summary>
        /// SAC-IA-3DSC配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源3DSC描述子集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标3DSC描述子集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "saciaAlign3DSC")]
        public static extern IntPtr SaciaAlign3DSC(Point3F[] sourcePoints, ShapeContext1980F[] sourceDescriptors, int sourceLength, Point3F[] targetPoints, ShapeContext1980F[] targetDescriptors, int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);
        #endregion

        #region # SAC-IA-SHOT配准 —— static extern IntPtr SaciaAlignSHOT(Point3F[] sourcePoints...
        /// <summary>
        /// SAC-IA-SHOT配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源SHOT描述子集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标SHOT描述子集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "saciaAlignSHOT")]
        public static extern IntPtr SaciaAlignSHOT(Point3F[] sourcePoints, Shot352F[] sourceDescriptors, int sourceLength, Point3F[] targetPoints, Shot352F[] targetDescriptors, int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);
        #endregion

        #region # ICP-Point-To-Point配准 —— static extern IntPtr AlignPointToPoint(Point3F[] sourcePoints...
        /// <summary>
        /// ICP-Point-To-Point配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="maxCorrespondenceDistance">分辨率</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "alignPointToPoint")]
        public static extern IntPtr AlignPointToPoint(Point3F[] sourcePoints, int sourceLength, Point3F[] targetPoints, int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);
        #endregion

        #region # ICP-Point-To-Plane配准 —— static extern IntPtr AlignPointToPlane(Point3F[] sourcePoints...
        /// <summary>
        /// ICP-Point-To-Plane配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceLength">源点集长度</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetLength">目标点集长度</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="maxCorrespondenceDistance">分辨率</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>配准结果</returns>
        [DllImport(AssemblyNames.Modules, EntryPoint = "alignPointToPlane")]
        public static extern IntPtr AlignPointToPlane(Point3F[] sourcePoints, int sourceLength, Point3F[] targetPoints, int targetLength, int normalK, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations, int threadsCount);
        #endregion
    }
}
