using PCLSharp.Modules.Declarations;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Features;
using PCLSharp.Primitives.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Implements
{
    /// <summary>
    /// 点云配准实现
    /// </summary>
    public class CloudRegistrations : ICloudRegistrations
    {
        #region # 4PCS配准 —— AlignmentResult Align4PCS(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// 4PCS配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="approxOverlap">近似重叠</param>
        /// <param name="delta">配准距离</param>
        /// <param name="normalize">标准化</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="maxComputationTime">最大计算时间(秒)</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult Align4PCS(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float approxOverlap, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.Align4PCS(sourcePoints_, sourcePoints_.Length, targetPoints_, targetPoints_.Length, approxOverlap, delta, normalize, samplesCount, maxComputationTime, threadsCount);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion

        #region # K-4PCS配准 —— AlignmentResult AlignK4PCS(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// K-4PCS配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="approxOverlap">近似重叠</param>
        /// <param name="lambda">平移向量加权系数</param>
        /// <param name="delta">配准距离</param>
        /// <param name="normalize">标准化</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="maxComputationTime">最大计算时间(秒)</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult AlignK4PCS(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float approxOverlap, float lambda, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.AlignK4PCS(sourcePoints_, sourcePoints_.Length, targetPoints_, targetPoints_.Length, approxOverlap, lambda, delta, normalize, samplesCount, maxComputationTime, threadsCount);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion

        #region # NDT配准 —— AlignmentResult AlignNDT(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// NDT配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="resolution">分辨率</param>
        /// <param name="stepSize">步长</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult AlignNDT(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float resolution, float stepSize, float transformationEpsilon, int maximumIterations)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.AlignNDT(sourcePoints_, sourcePoints_.Length, targetPoints_, targetPoints_.Length, resolution, stepSize, transformationEpsilon, maximumIterations);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion

        #region # ICP配准 —— AlignmentResult AlignICP(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// ICP配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="maxCorrespondenceDistance">分辨率</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult AlignICP(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.AlignICP(sourcePoints_, sourcePoints_.Length, targetPoints_, targetPoints_.Length, maxCorrespondenceDistance, transformationEpsilon, euclideanFitnessEpsilon, maximumIterations);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion

        #region # GICP配准 —— AlignmentResult AlignGICP(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// GICP配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="maxCorrespondenceDistance">分辨率</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult AlignGICP(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.AlignGICP(sourcePoints_, sourcePoints_.Length, targetPoints_, targetPoints_.Length, maxCorrespondenceDistance, transformationEpsilon, euclideanFitnessEpsilon, maximumIterations);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion

        #region # SAC-IA-NARF配准 —— AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// SAC-IA-NARF配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源NARF描述子集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标NARF描述子集</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<Narf36F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<Narf36F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();
            Narf36F[] sourceDescriptors_ = sourceDescriptors?.ToArray() ?? Array.Empty<Narf36F>();
            Narf36F[] targetDescriptors_ = targetDescriptors?.ToArray() ?? Array.Empty<Narf36F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }
            if (!sourceDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(sourceDescriptors), "源描述子集不可为空！");
            }
            if (!targetDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(targetDescriptors), "目标描述子集不可为空！");
            }
            if (sourcePoints_.Length != sourceDescriptors_.Length)
            {
                throw new InvalidOperationException("源点集与源描述子集长度不同！");
            }
            if (targetPoints_.Length != targetDescriptors_.Length)
            {
                throw new InvalidOperationException("目标点集与目标描述子集长度不同！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.SaciaAlignNARF(sourcePoints_, sourceDescriptors_, sourcePoints_.Length, targetPoints_, targetDescriptors_, targetPoints_.Length, minSampleDistance, samplesCount, correspondenceRandomness);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion

        #region # SAC-IA-PFH配准 —— AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// SAC-IA-PFH配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源PFH描述子集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标PFH描述子集</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<PFHSignature125F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<PFHSignature125F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();
            PFHSignature125F[] sourceDescriptors_ = sourceDescriptors?.ToArray() ?? Array.Empty<PFHSignature125F>();
            PFHSignature125F[] targetDescriptors_ = targetDescriptors?.ToArray() ?? Array.Empty<PFHSignature125F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }
            if (!sourceDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(sourceDescriptors), "源描述子集不可为空！");
            }
            if (!targetDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(targetDescriptors), "目标描述子集不可为空！");
            }
            if (sourcePoints_.Length != sourceDescriptors_.Length)
            {
                throw new InvalidOperationException("源点集与源描述子集长度不同！");
            }
            if (targetPoints_.Length != targetDescriptors_.Length)
            {
                throw new InvalidOperationException("目标点集与目标描述子集长度不同！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.SaciaAlignPFH(sourcePoints_, sourceDescriptors_, sourcePoints_.Length, targetPoints_, targetDescriptors_, targetPoints_.Length, minSampleDistance, samplesCount, correspondenceRandomness);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion

        #region # SAC-IA-FPFH配准 —— AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// SAC-IA-FPFH配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源FPFH描述子集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标FPFH描述子集</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<FPFHSignature33F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<FPFHSignature33F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();
            FPFHSignature33F[] sourceDescriptors_ = sourceDescriptors?.ToArray() ?? Array.Empty<FPFHSignature33F>();
            FPFHSignature33F[] targetDescriptors_ = targetDescriptors?.ToArray() ?? Array.Empty<FPFHSignature33F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }
            if (!sourceDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(sourceDescriptors), "源描述子集不可为空！");
            }
            if (!targetDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(targetDescriptors), "目标描述子集不可为空！");
            }
            if (sourcePoints_.Length != sourceDescriptors_.Length)
            {
                throw new InvalidOperationException("源点集与源描述子集长度不同！");
            }
            if (targetPoints_.Length != targetDescriptors_.Length)
            {
                throw new InvalidOperationException("目标点集与目标描述子集长度不同！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.SaciaAlignFPFH(sourcePoints_, sourceDescriptors_, sourcePoints_.Length, targetPoints_, targetDescriptors_, targetPoints_.Length, minSampleDistance, samplesCount, correspondenceRandomness);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion

        #region # SAC-IA-3DSC配准 —— AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// SAC-IA-3DSC配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源3DSC描述子集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标3DSC描述子集</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<ShapeContext1980F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<ShapeContext1980F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();
            ShapeContext1980F[] sourceDescriptors_ = sourceDescriptors?.ToArray() ?? Array.Empty<ShapeContext1980F>();
            ShapeContext1980F[] targetDescriptors_ = targetDescriptors?.ToArray() ?? Array.Empty<ShapeContext1980F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }
            if (!sourceDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(sourceDescriptors), "源描述子集不可为空！");
            }
            if (!targetDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(targetDescriptors), "目标描述子集不可为空！");
            }
            if (sourcePoints_.Length != sourceDescriptors_.Length)
            {
                throw new InvalidOperationException("源点集与源描述子集长度不同！");
            }
            if (targetPoints_.Length != targetDescriptors_.Length)
            {
                throw new InvalidOperationException("目标点集与目标描述子集长度不同！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.SaciaAlign3DSC(sourcePoints_, sourceDescriptors_, sourcePoints_.Length, targetPoints_, targetDescriptors_, targetPoints_.Length, minSampleDistance, samplesCount, correspondenceRandomness);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion

        #region # SAC-IA-SHOT配准 —— AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// SAC-IA-SHOT配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="sourceDescriptors">源SHOT描述子集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="targetDescriptors">目标SHOT描述子集</param>
        /// <param name="minSampleDistance">采样点最小距离</param>
        /// <param name="samplesCount">采样数量</param>
        /// <param name="correspondenceRandomness">随机特征邻域点数</param>
        /// <returns>配准结果</returns>
        public AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<Shot352F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<Shot352F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness)
        {
            Point3F[] sourcePoints_ = sourcePoints?.ToArray() ?? Array.Empty<Point3F>();
            Point3F[] targetPoints_ = targetPoints?.ToArray() ?? Array.Empty<Point3F>();
            Shot352F[] sourceDescriptors_ = sourceDescriptors?.ToArray() ?? Array.Empty<Shot352F>();
            Shot352F[] targetDescriptors_ = targetDescriptors?.ToArray() ?? Array.Empty<Shot352F>();

            #region # 验证

            if (!sourcePoints_.Any())
            {
                throw new ArgumentNullException(nameof(sourcePoints), "源点集不可为空！");
            }
            if (!targetPoints_.Any())
            {
                throw new ArgumentNullException(nameof(targetPoints), "目标点集不可为空！");
            }
            if (!sourceDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(sourceDescriptors), "源描述子集不可为空！");
            }
            if (!targetDescriptors_.Any())
            {
                throw new ArgumentNullException(nameof(targetDescriptors), "目标描述子集不可为空！");
            }
            if (sourcePoints_.Length != sourceDescriptors_.Length)
            {
                throw new InvalidOperationException("源点集与源描述子集长度不同！");
            }
            if (targetPoints_.Length != targetDescriptors_.Length)
            {
                throw new InvalidOperationException("目标点集与目标描述子集长度不同！");
            }

            #endregion

            IntPtr pointer = RegistrationsNative.SaciaAlignSHOT(sourcePoints_, sourceDescriptors_, sourcePoints_.Length, targetPoints_, targetDescriptors_, targetPoints_.Length, minSampleDistance, samplesCount, correspondenceRandomness);
            AlignmentResult alignmentResult = Marshal.PtrToStructure<AlignmentResult>(pointer);
            DisposeNative.DisposeAlignmentResult(pointer);

            return alignmentResult;
        }
        #endregion
    }
}
