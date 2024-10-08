﻿using PCLSharp.Primitives.Features;
using PCLSharp.Primitives.Models;
using System.Collections.Generic;

namespace PCLSharp.Modules.Interfaces
{
    /// <summary>
    /// 点云配准接口
    /// </summary>
    public interface ICloudRegistrations
    {
        #region # FPCS配准 —— AlignmentResult AlignFPCS(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// FPCS配准
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
        AlignmentResult AlignFPCS(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float approxOverlap, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);
        #endregion

        #region # K-FPCS配准 —— AlignmentResult AlignKFPCS(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// K-FPCS配准
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
        AlignmentResult AlignKFPCS(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float approxOverlap, float lambda, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);
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
        AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<Narf36F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<Narf36F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness);
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
        AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<PFHSignature125F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<PFHSignature125F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness);
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
        AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<FPFHSignature33F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<FPFHSignature33F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness);
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
        AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<ShapeContext1980F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<ShapeContext1980F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness);
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
        AlignmentResult AlignSACIA(IEnumerable<Point3F> sourcePoints, IEnumerable<Shot352F> sourceDescriptors, IEnumerable<Point3F> targetPoints, IEnumerable<Shot352F> targetDescriptors, float minSampleDistance, int samplesCount, int correspondenceRandomness);
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
        AlignmentResult AlignNDT(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float resolution, float stepSize, float transformationEpsilon, int maximumIterations);
        #endregion

        #region # GICP配准 —— AlignmentResult AlignGICP(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// GICP配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="maxCorrespondenceDistance">最大相似距离</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <returns>配准结果</returns>
        AlignmentResult AlignGICP(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);
        #endregion

        #region # ICP-Point-To-Point配准 —— AlignmentResult AlignPointToPoint(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// ICP-Point-To-Point配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="maxCorrespondenceDistance">最大相似距离</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <returns>配准结果</returns>
        AlignmentResult AlignPointToPoint(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);
        #endregion

        #region # ICP-Point-To-Plane配准 —— AlignmentResult AlignPointToPlane(IEnumerable<Point3F> sourcePoints...
        /// <summary>
        /// ICP-Point-To-Plane配准
        /// </summary>
        /// <param name="sourcePoints">源点集</param>
        /// <param name="targetPoints">目标点集</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="maxCorrespondenceDistance">最大相似距离</param>
        /// <param name="transformationEpsilon">变换最大差值</param>
        /// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
        /// <param name="maximumIterations">最大迭代次数</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>配准结果</returns>
        AlignmentResult AlignPointToPlane(IEnumerable<Point3F> sourcePoints, IEnumerable<Point3F> targetPoints, int normalK, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations, int threadsCount);
        #endregion
    }
}
