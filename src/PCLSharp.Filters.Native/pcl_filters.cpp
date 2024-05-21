#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
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
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//加载点云
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

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
/// 适用均匀采样
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="radius">采样半径</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyUniformSampling(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//加载点云
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

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
/// <param name="leafSize">叶尺寸</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyVoxelGrid(Point3F points[], const int length, const float leafSize)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//加载点云
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

	//体素降采样
	VoxelGrid<PointXYZ> voxelGrid;
	voxelGrid.setInputCloud(sourceCloud);
	voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
	voxelGrid.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 适用离群点移除
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="meanK">平均距离估计的最近邻居的数量</param>
/// <param name="stddevMult">标准差阈值系数</param>
/// <returns>过滤后点集</returns>
Point3Fs* applyOutlierRemoval(Point3F points[], const int length, const int meanK, const float stddevMult)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//加载点云
	const PointCloud<PointXYZ>& pointCloud = pclsharp::toPointCloud(points, length);
	copyPointCloud(pointCloud, *sourceCloud);

	//初始化统计学离群点移除过滤器
	StatisticalOutlierRemoval<PointXYZ> statisticalOutlierRemoval;
	statisticalOutlierRemoval.setInputCloud(sourceCloud);
	statisticalOutlierRemoval.setMeanK(50);//设置平均距离估计的最近邻居的数量K	
	statisticalOutlierRemoval.setStddevMulThresh(1.0);//设置标准差阈值系数
	statisticalOutlierRemoval.filter(*targetCloud);	//执行过滤

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 释放资源
/// </summary>
/// <param name="pointer">指针</param>
void dispose(const Point3Fs* pointer)
{
	delete pointer;
}
