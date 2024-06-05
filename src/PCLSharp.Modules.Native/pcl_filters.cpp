#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <primitives_map.h>
#include "pcl_filters.h"
using namespace std;
using namespace pcl;

/// <summary>
/// 适用直通滤波
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="axis">过滤坐标轴</param>
/// <param name="limitMin">过滤范围最小值</param>
/// <param name="limixMax">过滤范围最大值</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyPassThrogh(Point3F points[], const int length, const char* axis, const float limitMin, const float limixMax)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//直通滤波
	PassThrough<PointXYZ> passThrough;
	passThrough.setInputCloud(sourceCloud);
	passThrough.setFilterFieldName(axis);				//设置过滤坐标轴
	passThrough.setFilterLimits(limitMin, limixMax);	//设置过滤范围
	passThrough.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 适用随机采样
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="seed">随机种子</param>
/// <param name="samplesCount">采样数量</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyRandomSampling(Point3F points[], const int length, const int seed, const int samplesCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//随机采样
	RandomSample<PointXYZ> randomSampling;
	randomSampling.setInputCloud(sourceCloud);
	randomSampling.setSeed(seed);
	randomSampling.setSample(samplesCount);
	randomSampling.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 适用均匀采样
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="radius">采样半径</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyUniformSampling(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//均匀采样
	UniformSampling<PointXYZ> uniformSampling;
	uniformSampling.setInputCloud(sourceCloud);
	uniformSampling.setRadiusSearch(radius);
	uniformSampling.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 适用体素降采样
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="leafSize">网格尺寸</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyVoxelGrid(Point3F points[], const int length, const float leafSize)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//体素降采样
	VoxelGrid<PointXYZ> voxelGrid;
	voxelGrid.setInputCloud(sourceCloud);
	voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
	voxelGrid.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 适用近似体素降采样
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="leafSize">网格尺寸</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyApproxVoxelGrid(Point3F points[], int length, float leafSize)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//近似体素降采样
	ApproximateVoxelGrid<PointXYZ> voxelGrid;
	voxelGrid.setInputCloud(sourceCloud);
	voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
	voxelGrid.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 适用统计离群点移除
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="meanK">平均距离估计的最近邻居的数量</param>
/// <param name="stddevMult">标准差阈值系数</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyStatisticalOutlierRemoval(Point3F points[], const int length, const int meanK, const float stddevMult)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//统计离群点移除
	StatisticalOutlierRemoval<PointXYZ> statisticalOutlierRemoval;
	statisticalOutlierRemoval.setInputCloud(sourceCloud);
	statisticalOutlierRemoval.setMeanK(meanK);
	statisticalOutlierRemoval.setStddevMulThresh(stddevMult);
	statisticalOutlierRemoval.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 适用半径离群点移除
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="radius">搜索半径</param>
/// <param name="minNeighborsInRadius">半径范围内点数量最小值</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyRadiusOutlierRemoval(Point3F points[], int length, const float radius, const int minNeighborsInRadius)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//半径离群点移除
	RadiusOutlierRemoval<PointXYZ> radiusOutlierRemoval;
	radiusOutlierRemoval.setInputCloud(sourceCloud);
	radiusOutlierRemoval.setRadiusSearch(radius);
	radiusOutlierRemoval.setMinNeighborsInRadius(minNeighborsInRadius);
	radiusOutlierRemoval.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}
