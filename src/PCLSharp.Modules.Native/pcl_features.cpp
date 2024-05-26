#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <primitives_map.h>
#include <features_map.h>
#include "pcl_features.h"
using namespace std;
using namespace pcl;

/// <summary>
/// 计算NARF特征
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="angularResolution">角度分辨率</param>
/// <param name="maxAngleWidth">传感器水平边界角度</param>
/// <param name="maxAngleHeight">传感器垂直边界角度</param>
/// <param name="noiseLevel">所有与最近点的最大距离</param>
/// <param name="minRange">最小可见范围</param>
/// <param name="borderSize">边界尺寸</param>
/// <param name="supportSize">计算范围半径</param>
/// <param name="rotationInvariant">旋转不变性</param>
/// <returns>NARF特征描述子集</returns>
/// <remarks>特征K值要大于法向量K值</remarks>
Narf36Fs* computeNARF(Point3F points[], const int length, const float angularResolution, const float maxAngleWidth, const float maxAngleHeight, const float noiseLevel, const float minRange, const int borderSize, const float supportSize, const bool rotationInvariant)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();
	const RangeImage::Ptr rangeImage = std::make_shared<RangeImage>();

	//生成深度图
	const Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Affine3f::Identity());
	rangeImage->createFromPointCloud(*cloud, deg2rad(angularResolution), deg2rad(maxAngleWidth), deg2rad(maxAngleHeight), sensorPose, RangeImage::CAMERA_FRAME, noiseLevel, minRange, borderSize);

	//提取NARF关键点索引
	PointCloud<int> keyPointsIndices;
	RangeImageBorderExtractor rangeImageBorderExtractor;
	NarfKeypoint narfDetector = NarfKeypoint(&rangeImageBorderExtractor);
	narfDetector.setRangeImage(&*rangeImage);
	NarfKeypoint::Parameters& detectorParameters = narfDetector.getParameters();
	detectorParameters.support_size = supportSize;
	narfDetector.compute(keyPointsIndices);

	//复制NARF关键点索引
	vector<int> keyPointsIndicesV;
	keyPointsIndicesV.resize(keyPointsIndices.points.size());
	for (int i = 0; i < keyPointsIndices.size(); i++)
	{
		keyPointsIndicesV[i] = keyPointsIndices.points[i];
	}

	//提取NARF描述子
	PointCloud<Narf36> descriptors;
	NarfDescriptor narfComputer = NarfDescriptor(&*rangeImage, &keyPointsIndicesV);
	NarfDescriptor::Parameters& computerParameters = narfComputer.getParameters();
	computerParameters.support_size = supportSize;
	computerParameters.rotation_invariant = rotationInvariant;
	narfComputer.compute(descriptors);

	Narf36Fs* narf36Fs = pclsharp::toNarf36Fs(descriptors);

	return narf36Fs;
}

/// <summary>
/// 计算PFH特征
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="featureK">特征K</param>
/// <param name="threadsCount">线程数</param>
/// <returns>PFH特征描述子集</returns>
PFHSignature125Fs* computePFH(Point3F points[], const int length, const int normalK, const int featureK, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<PFHSignature125>::Ptr descriptors = std::make_shared<PointCloud<PFHSignature125>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//提取PFH描述子
	PFHEstimation<PointXYZ, Normal> pfhComputer;
	pfhComputer.setInputCloud(cloud);
	pfhComputer.setInputNormals(normals);
	pfhComputer.setSearchMethod(kdTree);
	pfhComputer.setKSearch(featureK);
	pfhComputer.compute(*descriptors);

	PFHSignature125Fs* signature125Fs = pclsharp::toPFHSignature125Fs(*descriptors);

	return signature125Fs;
}

/// <summary>
/// 计算FPFH特征
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="featureK">特征K</param>
/// <param name="threadsCount">线程数</param>
/// <returns>FPFH特征描述子集</returns>
/// <remarks>特征K值要大于法向量K值</remarks>
FPFHSignature33Fs* computeFPFH(Point3F points[], const int length, const int normalK, const int featureK, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<FPFHSignature33>::Ptr descriptors = std::make_shared<PointCloud<FPFHSignature33>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//提取FPFH描述子
	FPFHEstimationOMP<PointXYZ, Normal, FPFHSignature33> fpfhComputer;
	fpfhComputer.setInputCloud(cloud);
	fpfhComputer.setInputNormals(normals);
	fpfhComputer.setSearchMethod(kdTree);
	fpfhComputer.setKSearch(featureK);
	fpfhComputer.setNumberOfThreads(threadsCount);
	fpfhComputer.compute(*descriptors);

	FPFHSignature33Fs* signature33Fs = pclsharp::toFPFHSignature33Fs(*descriptors);

	return signature33Fs;
}

/// <summary>
/// 计算3DSC特征
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="searchRadius">搜索半径</param>
/// <param name="pointDensityRadius">点密度半径</param>
/// <param name="minimalRadius">最小半径</param>
/// <param name="threadsCount">线程数</param>
/// <returns>3DSC特征描述子集</returns>
/// <remarks>
/// 点密度半径建议设置为搜索半径的1/5；
///	最小半径建议设置为搜索半径的1/10；
/// </remarks>
ShapeContext1980Fs* compute3DSC(Point3F points[], const int length, const int normalK, const float searchRadius, const float pointDensityRadius, const float minimalRadius, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<ShapeContext1980>::Ptr descriptors = std::make_shared<PointCloud<ShapeContext1980>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//提取3DSC特征描述子
	ShapeContext3DEstimation<PointXYZ, Normal, ShapeContext1980> dsc3Computer;
	dsc3Computer.setInputCloud(cloud);
	dsc3Computer.setInputNormals(normals);
	dsc3Computer.setSearchMethod(kdTree);
	dsc3Computer.setRadiusSearch(searchRadius);
	dsc3Computer.setPointDensityRadius(pointDensityRadius);
	dsc3Computer.setMinimalRadius(minimalRadius);
	dsc3Computer.compute(*descriptors);

	ShapeContext1980Fs* shapeContext1980Fs = pclsharp::toShapeContext1980Fs(*descriptors);

	return shapeContext1980Fs;
}

/// <summary>
/// 计算SHOT特征
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="featureRadius">特征搜索半径</param>
/// <param name="threadsCount">线程数</param>
/// <returns>SHOT特征描述子集</returns>
EXPORT_C Shot352Fs* CALLING_MODE computeSHOT(Point3F points[], const int length, const int normalK, const float featureRadius, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<SHOT352>::Ptr descriptors = std::make_shared<PointCloud<SHOT352>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//提取SHOT特征描述子
	SHOTEstimationOMP<PointXYZ, Normal> shotComputer;
	shotComputer.setInputCloud(cloud);
	shotComputer.setInputNormals(normals);
	shotComputer.setRadiusSearch(featureRadius);
	shotComputer.setNumberOfThreads(threadsCount);
	shotComputer.compute(*descriptors);

	Shot352Fs* shot352Fs = pclsharp::toShot352Fs(*descriptors);

	return shot352Fs;
}
