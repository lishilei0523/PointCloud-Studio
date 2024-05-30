#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <narf36f.h>
#include <pfh_signature125f.h>
#include <fpfh_signature33f.h>
#include <shape_context1980f.h>
#include <shot352f.h>
#include <alignment_result.h>

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
EXPORT_C AlignmentResult* CALLING_MODE align4PCS(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float approxOverlap, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);

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
EXPORT_C AlignmentResult* CALLING_MODE alignK4PCS(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float approxOverlap, float lambda, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);

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
EXPORT_C AlignmentResult* CALLING_MODE alignNDT(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float resolution, float stepSize, float transformationEpsilon, int maximumIterations);

/// <summary>
/// ICP配准
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
EXPORT_C AlignmentResult* CALLING_MODE alignICP(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);

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
EXPORT_C AlignmentResult* CALLING_MODE alignGICP(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);

/// <summary>
/// SAC-IA&NARF配准
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
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlignNARF(Point3F sourcePoints[], Narf36F sourceDescriptors[], int sourceLength, Point3F targetPoints[], Narf36F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

/// <summary>
/// SAC-IA&PFH配准
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
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlignPFH(Point3F sourcePoints[], PFHSignature125F sourceDescriptors[], int sourceLength, Point3F targetPoints[], PFHSignature125F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

/// <summary>
/// SAC-IA&FPFH配准
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
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlignFPFH(Point3F sourcePoints[], FPFHSignature33F sourceDescriptors[], int sourceLength, Point3F targetPoints[], FPFHSignature33F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

/// <summary>
/// SAC-IA&3DSC配准
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
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlign3DSC(Point3F sourcePoints[], ShapeContext1980F sourceDescriptors[], int sourceLength, Point3F targetPoints[], ShapeContext1980F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

/// <summary>
/// SAC-IA&SHOT配准
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
EXPORT_C AlignmentResult* CALLING_MODE SaciaAlignSHOT(Point3F sourcePoints[], Shot352F sourceDescriptors[], int sourceLength, Point3F targetPoints[], Shot352F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);
