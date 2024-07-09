#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#elif __linux__
#define EXPORT_C extern "C"
#define CALLING_MODE __attribute__((__cdecl__))
#endif
#include <point3f.h>
#include <narf36f.h>
#include <pfh_signature125f.h>
#include <fpfh_signature33f.h>
#include <shape_context1980f.h>
#include <shot352f.h>
#include <alignment_result.h>

/// <summary>
/// FPCS配准
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
EXPORT_C AlignmentResult* CALLING_MODE alignFPCS(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float approxOverlap, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);

/// <summary>
/// K-FPCS配准
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
EXPORT_C AlignmentResult* CALLING_MODE alignKFPCS(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float approxOverlap, float lambda, float delta, bool normalize, int samplesCount, int maxComputationTime, int threadsCount);

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
EXPORT_C AlignmentResult* CALLING_MODE saciaAlignNARF(Point3F sourcePoints[], Narf36F sourceDescriptors[], int sourceLength, Point3F targetPoints[], Narf36F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

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
EXPORT_C AlignmentResult* CALLING_MODE saciaAlignPFH(Point3F sourcePoints[], PFHSignature125F sourceDescriptors[], int sourceLength, Point3F targetPoints[], PFHSignature125F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

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
EXPORT_C AlignmentResult* CALLING_MODE saciaAlignFPFH(Point3F sourcePoints[], FPFHSignature33F sourceDescriptors[], int sourceLength, Point3F targetPoints[], FPFHSignature33F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

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
EXPORT_C AlignmentResult* CALLING_MODE saciaAlign3DSC(Point3F sourcePoints[], ShapeContext1980F sourceDescriptors[], int sourceLength, Point3F targetPoints[], ShapeContext1980F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

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
EXPORT_C AlignmentResult* CALLING_MODE saciaAlignSHOT(Point3F sourcePoints[], Shot352F sourceDescriptors[], int sourceLength, Point3F targetPoints[], Shot352F targetDescriptors[], int targetLength, float minSampleDistance, int samplesCount, int correspondenceRandomness);

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
/// GICP配准
/// </summary>
/// <param name="sourcePoints">源点集</param>
/// <param name="sourceLength">源点集长度</param>
/// <param name="targetPoints">目标点集</param>
/// <param name="targetLength">目标点集长度</param>
/// <param name="maxCorrespondenceDistance">最大相似距离</param>
/// <param name="transformationEpsilon">变换最大差值</param>
/// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
/// <param name="maximumIterations">最大迭代次数</param>
/// <returns>配准结果</returns>
EXPORT_C AlignmentResult* CALLING_MODE alignGICP(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);

/// <summary>
/// ICP-Point-To-Point配准
/// </summary>
/// <param name="sourcePoints">源点集</param>
/// <param name="sourceLength">源点集长度</param>
/// <param name="targetPoints">目标点集</param>
/// <param name="targetLength">目标点集长度</param>
/// <param name="maxCorrespondenceDistance">最大相似距离</param>
/// <param name="transformationEpsilon">变换最大差值</param>
/// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
/// <param name="maximumIterations">最大迭代次数</param>
/// <returns>配准结果</returns>
EXPORT_C AlignmentResult* CALLING_MODE alignPointToPoint(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations);

/// <summary>
/// ICP-Point-To-Plane配准
/// </summary>
/// <param name="sourcePoints">源点集</param>
/// <param name="sourceLength">源点集长度</param>
/// <param name="targetPoints">目标点集</param>
/// <param name="targetLength">目标点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="maxCorrespondenceDistance">最大相似距离</param>
/// <param name="transformationEpsilon">变换最大差值</param>
/// <param name="euclideanFitnessEpsilon">均方误差阈值</param>
/// <param name="maximumIterations">最大迭代次数</param>
/// <param name="threadsCount">线程数</param>
/// <returns>配准结果</returns>
EXPORT_C AlignmentResult* CALLING_MODE alignPointToPlane(Point3F sourcePoints[], int sourceLength, Point3F targetPoints[], int targetLength, int normalK, float maxCorrespondenceDistance, float transformationEpsilon, float euclideanFitnessEpsilon, int maximumIterations, int threadsCount);
