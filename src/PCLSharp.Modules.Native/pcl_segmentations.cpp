#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/angles.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <primitives_map.h>
#include "pcl_segmentations.h"
using namespace std;
using namespace pcl;
using namespace pcl::search;

/// <summary>
/// 分割平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="optimizeCoefficients">是否优化模型系数</param>
/// <param name="probability">概率</param>
/// <param name="distanceThreshold">距离阈值</param>
/// <param name="maxIterationsCount">最大迭代次数</param>
/// <param name="a">平面方程系数a</param>
/// <param name="b">平面方程系数b</param>
/// <param name="c">平面方程系数c</param>
/// <param name="d">平面方程系数d</param>
/// <returns>平面点云</returns>
Point3Fs* segmentPlane(Point3F points[], const int length, const bool optimizeCoefficients, const float probability, const float distanceThreshold, const int maxIterationsCount, int& a, int& b, int& c, int& d)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//分割平面
	PointIndices inliers;
	ModelCoefficients coefficients;
	SACSegmentation<PointXYZ> sacSegmenter;
	sacSegmenter.setInputCloud(sourceCloud);
	sacSegmenter.setOptimizeCoefficients(optimizeCoefficients);
	sacSegmenter.setModelType(SACMODEL_PLANE);
	sacSegmenter.setMethodType(SAC_RANSAC);
	sacSegmenter.setProbability(probability);
	sacSegmenter.setDistanceThreshold(distanceThreshold);
	sacSegmenter.setMaxIterations(maxIterationsCount);
	sacSegmenter.segment(inliers, coefficients);
	pcl::copyPointCloud(*sourceCloud, inliers, *targetCloud);
	if (!coefficients.values.empty())
	{
		a = coefficients.values[0];
		b = coefficients.values[1];
		c = coefficients.values[2];
		d = coefficients.values[3];
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 分割球体
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="optimizeCoefficients">是否优化模型系数</param>
/// <param name="probability">概率</param>
/// <param name="distanceThreshold">距离阈值</param>
/// <param name="minRadius">球体最小半径</param>
/// <param name="maxRadius">球体最大半径</param>
/// <param name="maxIterationsCount">最大迭代次数</param>
/// <returns>球体点云</returns>
Point3Fs* segmentSphere(Point3F points[], const int length, const bool optimizeCoefficients, const float probability, const float distanceThreshold, const float minRadius, const float maxRadius, const int maxIterationsCount)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//分割球体
	PointIndices inliers;
	ModelCoefficients coefficients;
	SACSegmentation<PointXYZ> sacSegmenter;
	sacSegmenter.setInputCloud(sourceCloud);
	sacSegmenter.setOptimizeCoefficients(optimizeCoefficients);
	sacSegmenter.setModelType(SACMODEL_SPHERE);
	sacSegmenter.setMethodType(SAC_RANSAC);
	sacSegmenter.setProbability(probability);
	sacSegmenter.setDistanceThreshold(distanceThreshold);
	sacSegmenter.setRadiusLimits(minRadius, maxRadius);
	sacSegmenter.setMaxIterations(maxIterationsCount);
	sacSegmenter.segment(inliers, coefficients);
	pcl::copyPointCloud(*sourceCloud, inliers, *targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 欧几里得聚类分割
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="clusterTolerance">簇搜索容差</param>
/// <param name="minClusterSize">簇最小尺寸</param>
/// <param name="maxClusterSize">簇最大尺寸</param>
/// <param name="clustersCount">点云簇数</param>
/// <returns>点云簇列表</returns>
Point3Fs** euclidClusterSegment(Point3F points[], const int length, const float clusterTolerance, const int minClusterSize, const int maxClusterSize, int& clustersCount)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//欧几里得聚类分割
	vector<PointIndices> clusterIndices;
	EuclideanClusterExtraction<PointXYZ> clusterExtractor;
	clusterExtractor.setInputCloud(cloud);
	clusterExtractor.setSearchMethod(kdTree);
	clusterExtractor.setClusterTolerance(clusterTolerance);
	clusterExtractor.setMinClusterSize(minClusterSize);
	clusterExtractor.setMaxClusterSize(maxClusterSize);
	clusterExtractor.extract(clusterIndices);

	//分离簇点云
	clustersCount = static_cast<int>(clusterIndices.size());
	Point3Fs** pointsGroup = new Point3Fs * [clusterIndices.size()];
	for (int clusterIndex = 0; clusterIndex < clustersCount; clusterIndex++)
	{
		const PointIndices& pointIndices = clusterIndices[clusterIndex];
		const PointIndices::Ptr inliers = std::make_shared<PointIndices>(pointIndices);
		PointCloud<PointXYZ> cloudCluster;
		ExtractIndices<PointXYZ> extractor;
		extractor.setInputCloud(cloud);
		extractor.setIndices(inliers);
		extractor.setNegative(false);
		extractor.filter(cloudCluster);
		pointsGroup[clusterIndex] = pclsharp::toPoint3Fs(cloudCluster);
	}

	return pointsGroup;
}

/// <summary>
/// 区域生长分割
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="clusterK">簇K</param>
/// <param name="smoothnessThreshold">平滑阈值（角度）</param>
/// <param name="curvatureThreshold">曲率阈值</param>
/// <param name="minClusterSize">簇最小尺寸</param>
/// <param name="maxClusterSize">簇最大尺寸</param>
/// <param name="threadsCount">线程数</param>
/// <param name="clustersCount">点云簇数</param>
/// <returns>点云簇列表</returns>
Point3Fs** regionGrowingSegment(Point3F points[], const int length, const int normalK, const int clusterK, const float smoothnessThreshold, const float curvatureThreshold, const int minClusterSize, const int maxClusterSize, const int threadsCount, int& clustersCount)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//区域生长分割
	vector<PointIndices> clusterIndices;
	RegionGrowing<PointXYZ, Normal> rgSegmenter;
	rgSegmenter.setInputCloud(cloud);
	rgSegmenter.setInputNormals(normals);
	rgSegmenter.setSearchMethod(kdTree);
	rgSegmenter.setNumberOfNeighbours(clusterK);
	rgSegmenter.setSmoothnessThreshold(deg2rad(smoothnessThreshold));
	rgSegmenter.setCurvatureThreshold(curvatureThreshold);
	rgSegmenter.setMinClusterSize(minClusterSize);
	rgSegmenter.setMaxClusterSize(maxClusterSize);
	rgSegmenter.extract(clusterIndices);

	//分离簇点云
	clustersCount = static_cast<int>(clusterIndices.size());
	Point3Fs** pointsGroup = new Point3Fs * [clusterIndices.size()];
	for (int clusterIndex = 0; clusterIndex < clustersCount; clusterIndex++)
	{
		const PointIndices& pointIndices = clusterIndices[clusterIndex];
		const PointIndices::Ptr inliers = std::make_shared<PointIndices>(pointIndices);
		PointCloud<PointXYZ> cloudCluster;
		ExtractIndices<PointXYZ> extractor;
		extractor.setInputCloud(cloud);
		extractor.setIndices(inliers);
		extractor.setNegative(false);
		extractor.filter(cloudCluster);
		pointsGroup[clusterIndex] = pclsharp::toPoint3Fs(cloudCluster);
	}

	return pointsGroup;
}

/// <summary>
/// 区域生长颜色分割
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="clusterK">簇K</param>
/// <param name="distanceThreshold">距离阈值</param>
/// <param name="smoothnessThreshold">平滑阈值（角度）</param>
/// <param name="curvatureThreshold">曲率阈值</param>
/// <param name="pointColorThreshold">点颜色阈值</param>
/// <param name="regionColorThreshold">区域颜色阈值</param>
/// <param name="minClusterSize">簇最小尺寸</param>
/// <param name="maxClusterSize">簇最大尺寸</param>
/// <param name="threadsCount">线程数</param>
/// <param name="clustersCount">点云簇数</param>
/// <returns>点云簇列表</returns>
Point3Color4s** regionGrowingColorSegment(Point3Color4 points[], const int length, const int normalK, const int clusterK, const float distanceThreshold, const float smoothnessThreshold, const float curvatureThreshold, const float pointColorThreshold, const float regionColorThreshold, const int minClusterSize, const int maxClusterSize, const int threadsCount, int& clustersCount)
{
	const PointCloud<PointXYZRGB>::Ptr cloud = pclsharp::toPointCloudRGB(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const search::KdTree<PointXYZRGB>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZRGB>>();

	//计算法向量
	NormalEstimationOMP<PointXYZRGB, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//区域生长颜色分割
	vector<PointIndices> clusterIndices;
	RegionGrowingRGB<PointXYZRGB> rgSegmenter;
	rgSegmenter.setInputCloud(cloud);
	rgSegmenter.setInputNormals(normals);
	rgSegmenter.setSearchMethod(kdTree);
	rgSegmenter.setNumberOfNeighbours(clusterK);
	rgSegmenter.setDistanceThreshold(distanceThreshold);
	rgSegmenter.setSmoothnessThreshold(deg2rad(smoothnessThreshold));
	rgSegmenter.setCurvatureThreshold(curvatureThreshold);
	rgSegmenter.setPointColorThreshold(pointColorThreshold);
	rgSegmenter.setRegionColorThreshold(regionColorThreshold);
	rgSegmenter.setMinClusterSize(minClusterSize);
	rgSegmenter.setMaxClusterSize(maxClusterSize);
	rgSegmenter.extract(clusterIndices);

	//分离簇点云
	clustersCount = static_cast<int>(clusterIndices.size());
	Point3Color4s** pointsGroup = new Point3Color4s * [clusterIndices.size()];
	for (int clusterIndex = 0; clusterIndex < clustersCount; clusterIndex++)
	{
		const PointIndices& pointIndices = clusterIndices[clusterIndex];
		const PointIndices::Ptr inliers = std::make_shared<PointIndices>(pointIndices);
		PointCloud<PointXYZRGB> cloudCluster;
		ExtractIndices<PointXYZRGB> extractor;
		extractor.setInputCloud(cloud);
		extractor.setIndices(inliers);
		extractor.setNegative(false);
		extractor.filter(cloudCluster);
		pointsGroup[clusterIndex] = pclsharp::toPoint3Color4s(cloudCluster);
	}

	return pointsGroup;
}
