#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/extract_indices.h>
#include <primitives_map.h>
#include "pcl_search.h"
using namespace std;
using namespace pcl;
using namespace pcl::octree;

/// <summary>
/// K近邻搜索
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="referencePoint">参考坐标点</param>
/// <param name="k">近邻数量</param>
/// <returns>结果点集</returns>
Point3Fs* kSearch(Point3F points[], const int length, const Point3F referencePoint, const int k)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointXYZ& point = pclsharp::toPointXYZ(referencePoint);

	const IndicesPtr pointIndices = std::make_shared<Indices>();	//搜索到的近邻点的索引
	vector<float> pointSqrDistances;								//参考点对应近邻点的距离的平方
	search::KdTree<PointXYZ> kdTree;
	kdTree.setInputCloud(sourceCloud);
	const int& neighborsCount = kdTree.nearestKSearch(point, k, *pointIndices, pointSqrDistances);
	if (neighborsCount > 0)
	{
		ExtractIndices<PointXYZ> extractor;
		extractor.setInputCloud(sourceCloud);
		extractor.setIndices(pointIndices);
		extractor.setNegative(false);
		extractor.filter(*targetCloud);
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 半径搜索
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="referencePoint">参考坐标点</param>
/// <param name="radius">搜索半径</param>
/// <returns>结果点集</returns>
Point3Fs* CALLING_MODE radiusSearch(Point3F points[], const int length, const Point3F referencePoint, const float radius)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointXYZ& point = pclsharp::toPointXYZ(referencePoint);

	const IndicesPtr pointIndices = std::make_shared<Indices>();	//搜索到的近邻点的索引
	vector<float> pointSqrDistances;								//参考点对应近邻点的距离的平方
	search::KdTree<PointXYZ> kdTree;
	kdTree.setInputCloud(sourceCloud);
	const int& neighborsCount = kdTree.radiusSearch(point, radius, *pointIndices, pointSqrDistances);
	if (neighborsCount > 0)
	{
		ExtractIndices<PointXYZ> extractor;
		extractor.setInputCloud(sourceCloud);
		extractor.setIndices(pointIndices);
		extractor.setNegative(false);
		extractor.filter(*targetCloud);
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 八叉树搜索
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="referencePoint">参考坐标点</param>
/// <param name="resolution">分辨率</param>
/// <returns>结果点集</returns>
Point3Fs* octreeSearch(Point3F points[], const int length, const Point3F referencePoint, const float resolution)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointXYZ& point = pclsharp::toPointXYZ(referencePoint);

	const IndicesPtr pointIndices = std::make_shared<Indices>();	//搜索到的近邻点的索引
	OctreePointCloudSearch<PointXYZ> octree = OctreePointCloudSearch<PointXYZ>(resolution);
	octree.setInputCloud(sourceCloud);
	octree.addPointsFromInputCloud();
	const int& neighborsCount = octree.voxelSearch(point, *pointIndices);
	if (neighborsCount > 0)
	{
		ExtractIndices<PointXYZ> extractor;
		extractor.setInputCloud(sourceCloud);
		extractor.setIndices(pointIndices);
		extractor.setNegative(false);
		extractor.filter(*targetCloud);
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}
