#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <primitives_map.h>
#include "pcl_normals.h"
using namespace pcl;

/// <summary>
/// 估算法向量
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="k">搜索近邻数量</param>
/// <returns>法向量集</returns>
Normal3Fs* estimateNormalsByK(Point3F points[], const int length, const int k)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<Normal>::Ptr cloudNormals = std::make_shared<PointCloud<Normal>>();

	//加载点云
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

	//计算法向量
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	NormalEstimation<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(sourceCloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(k);
	normalEstimator.compute(*cloudNormals);

	Normal3Fs* normal3Fs = pclsharp::toNormal3Fs(*cloudNormals);

	return normal3Fs;
}

/// <summary>
/// 估算法向量
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="radius">搜索半径</param>
/// <returns>法向量集</returns>
Normal3Fs* estimateNormalsByRadius(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<Normal>::Ptr cloudNormals = std::make_shared<PointCloud<Normal>>();

	//加载点云
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

	//计算法向量
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	NormalEstimation<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(sourceCloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setRadiusSearch(radius);
	normalEstimator.compute(*cloudNormals);

	Normal3Fs* normal3Fs = pclsharp::toNormal3Fs(*cloudNormals);

	return normal3Fs;
}

/// <summary>
/// 估算法向量 (OMP)
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="k">搜索近邻数量</param>
/// <returns>法向量集</returns>
Normal3Fs* estimateNormalsByKP(Point3F points[], const int length, const int k)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<Normal>::Ptr cloudNormals = std::make_shared<PointCloud<Normal>>();

	//加载点云
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

	//计算法向量
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(sourceCloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(k);
	normalEstimator.compute(*cloudNormals);

	Normal3Fs* normal3Fs = pclsharp::toNormal3Fs(*cloudNormals);

	return normal3Fs;
}

/// <summary>
/// 估算法向量 (OMP)
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="radius">搜索半径</param>
/// <returns>法向量集</returns>
Normal3Fs* estimateNormalsByRadiusP(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<Normal>::Ptr cloudNormals = std::make_shared<PointCloud<Normal>>();

	//加载点云
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

	//计算法向量
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(sourceCloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setRadiusSearch(radius);
	normalEstimator.compute(*cloudNormals);

	Normal3Fs* normal3Fs = pclsharp::toNormal3Fs(*cloudNormals);

	return normal3Fs;
}

/// <summary>
/// 释放资源
/// </summary>
/// <param name="pointer">指针</param>
void dispose(const int* pointer)
{
	delete pointer;
}
