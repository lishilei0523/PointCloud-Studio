#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <primitives_map.h>
#include <features_map.h>
#include "pcl_registrations.h"
using namespace std;
using namespace pcl;
using namespace pcl::registration;

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
AlignmentResult* alignFPCS(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float approxOverlap, const float delta, const bool normalize, const int samplesCount, const int maxComputationTime, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//FPCS配准
	PointCloud<PointXYZ> finalCloud;
	FPCSInitialAlignment<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setApproxOverlap(approxOverlap);
	alignment.setDelta(delta, normalize);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setMaxComputationTime(maxComputationTime);
	alignment.setNumberOfThreads(threadsCount);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* alignKFPCS(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float approxOverlap, const float lambda, const float delta, const bool normalize, const int samplesCount, const int maxComputationTime, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//K-FPCS配准
	PointCloud<PointXYZ> finalCloud;
	KFPCSInitialAlignment<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setApproxOverlap(approxOverlap);
	alignment.setLambda(lambda);
	alignment.setDelta(delta, normalize);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setMaxComputationTime(maxComputationTime);
	alignment.setNumberOfThreads(threadsCount);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* saciaAlignNARF(Point3F sourcePoints[], Narf36F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], Narf36F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<Narf36>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<Narf36>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&NARF配准
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, Narf36> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* saciaAlignPFH(Point3F sourcePoints[], PFHSignature125F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], PFHSignature125F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<PFHSignature125>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<PFHSignature125>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&PFH配准
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, PFHSignature125> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* saciaAlignFPFH(Point3F sourcePoints[], FPFHSignature33F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], FPFHSignature33F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<FPFHSignature33>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<FPFHSignature33>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&FPFH配准
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, FPFHSignature33> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* saciaAlign3DSC(Point3F sourcePoints[], ShapeContext1980F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], ShapeContext1980F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<ShapeContext1980>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<ShapeContext1980>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&3DSC配准
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, ShapeContext1980> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* saciaAlignSHOT(Point3F sourcePoints[], Shot352F sourceDescriptors[], const int sourceLength, Point3F targetPoints[], Shot352F targetDescriptors[], const int targetLength, const float minSampleDistance, const int samplesCount, const int correspondenceRandomness)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<SHOT352>::Ptr& sourceFeatures = pclsharp::toPointCloud(sourceDescriptors, sourceLength);
	const PointCloud<SHOT352>::Ptr& targetFeatures = pclsharp::toPointCloud(targetDescriptors, targetLength);

	//SAC-IA&SHOT配准
	PointCloud<PointXYZ> finalCloud;
	SampleConsensusInitialAlignment<PointXYZ, PointXYZ, SHOT352> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setSourceFeatures(sourceFeatures);
	alignment.setTargetFeatures(targetFeatures);
	alignment.setMinSampleDistance(minSampleDistance);
	alignment.setNumberOfSamples(samplesCount);
	alignment.setCorrespondenceRandomness(correspondenceRandomness);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* alignNDT(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float resolution, const float stepSize, const float transformationEpsilon, const int maximumIterations)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//NDT配准
	PointCloud<PointXYZ> finalCloud;
	NormalDistributionsTransform<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setResolution(resolution);
	alignment.setStepSize(stepSize);
	alignment.setTransformationEpsilon(transformationEpsilon);
	alignment.setMaximumIterations(maximumIterations);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* alignGICP(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float maxCorrespondenceDistance, const float transformationEpsilon, const float euclideanFitnessEpsilon, const int maximumIterations)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//GICP配准
	PointCloud<PointXYZ> finalCloud;
	GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
	alignment.setTransformationEpsilon(transformationEpsilon);
	alignment.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
	alignment.setMaximumIterations(maximumIterations);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* alignPointToPoint(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const float maxCorrespondenceDistance, const float transformationEpsilon, const float euclideanFitnessEpsilon, const int maximumIterations)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);

	//ICP配准
	PointCloud<PointXYZ> finalCloud;
	IterativeClosestPoint<PointXYZ, PointXYZ> alignment;
	alignment.setInputSource(sourceCloud);
	alignment.setInputTarget(targetCloud);
	alignment.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
	alignment.setTransformationEpsilon(transformationEpsilon);
	alignment.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
	alignment.setMaximumIterations(maximumIterations);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}

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
AlignmentResult* alignPointToPlane(Point3F sourcePoints[], const int sourceLength, Point3F targetPoints[], const int targetLength, const int normalK, const float maxCorrespondenceDistance, const float transformationEpsilon, const float euclideanFitnessEpsilon, const int maximumIterations, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(sourcePoints, sourceLength);
	const PointCloud<PointXYZ>::Ptr& targetCloud = pclsharp::toPointCloud(targetPoints, targetLength);
	const PointCloud<Normal>::Ptr sourceNormals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<Normal>::Ptr targetNormals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<PointNormal>::Ptr sourcePointNormals = std::make_shared<PointCloud<PointNormal>>();
	const PointCloud<PointNormal>::Ptr targetPointNormals = std::make_shared<PointCloud<PointNormal>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.setInputCloud(sourceCloud);
	normalEstimator.compute(*sourceNormals);
	normalEstimator.setInputCloud(targetCloud);
	normalEstimator.compute(*targetNormals);
	pcl::concatenateFields(*sourceCloud, *sourceNormals, *sourcePointNormals);
	pcl::concatenateFields(*targetCloud, *targetNormals, *targetPointNormals);

	//ICP配准
	PointCloud<PointNormal> finalCloud;
	IterativeClosestPointWithNormals<PointNormal, PointNormal> alignment;
	alignment.setInputSource(sourcePointNormals);
	alignment.setInputTarget(targetPointNormals);
	alignment.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
	alignment.setTransformationEpsilon(transformationEpsilon);
	alignment.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon);
	alignment.setMaximumIterations(maximumIterations);
	alignment.align(finalCloud);

	//解析配准结果
	const bool& hasConverged = alignment.hasConverged();
	const float& fitnessScore = alignment.getFitnessScore();
	const Eigen::Matrix4f& rtMatrix = alignment.getFinalTransformation();
	AlignmentResult* alignmentResult = new AlignmentResult(hasConverged, fitnessScore);

	const int& rowsCount = rtMatrix.rows();
	const int& colsCount = rtMatrix.cols();
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const int& index = rowIndex * colsCount + colIndex;
			const float& value = rtMatrix(rowIndex, colIndex);
			alignmentResult->Matrix[index] = value;
		}
	}

	return alignmentResult;
}
