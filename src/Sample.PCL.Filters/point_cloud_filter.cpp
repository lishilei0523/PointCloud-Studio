#include "point_cloud_filter.h"
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
PointArray* applyPassThrogh(Point3F points[], const int length, const char* axis, const float limitMin, const float limixMax)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//加载点云
	for (int i = 0; i < length; i++)
	{
		const Point3F& point = points[i];
		PointXYZ pointXYZ = PointXYZ(point.X, point.Y, point.Z);
		sourceCloud->points.push_back(pointXYZ);
	}

	PassThrough<PointXYZ> passThrough;
	passThrough.setInputCloud(sourceCloud);
	passThrough.setFilterFieldName(axis);				//设置过滤坐标轴
	passThrough.setFilterLimits(limitMin, limixMax);	//设置过滤范围
	passThrough.filter(*targetCloud);

	const int outLength = targetCloud->size();
	PointArray* pointArray = new PointArray();
	pointArray->Length = outLength;
	pointArray->Points = new Point3F[outLength];
	for (int i = 0; i < outLength; i++)
	{
		const PointXYZ& pointXYZ = targetCloud->points[i];
		pointArray->Points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	return pointArray;
}

/// <summary>
/// 适用均匀采样
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="radius">采样半径</param>
/// <returns>过滤后点集</returns>
PointArray* applyUniformSampling(Point3F points[], const int length, const float radius)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//加载点云
	for (int i = 0; i < length; i++)
	{
		const Point3F& point = points[i];
		PointXYZ pointXYZ = PointXYZ(point.X, point.Y, point.Z);
		sourceCloud->points.push_back(pointXYZ);
	}

	//创建降采样过滤器对象
	UniformSampling<PointXYZ> uniformSampling;
	uniformSampling.setInputCloud(sourceCloud);
	uniformSampling.setRadiusSearch(radius);
	uniformSampling.filter(*targetCloud);

	const int outLength = targetCloud->size();
	PointArray* pointArray = new PointArray();
	pointArray->Length = outLength;
	pointArray->Points = new Point3F[outLength];
	for (int i = 0; i < outLength; i++)
	{
		const PointXYZ& pointXYZ = targetCloud->points[i];
		pointArray->Points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	return pointArray;
}

/// <summary>
/// 适用体素降采样
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="leafSize">叶尺寸</param>
/// <returns>过滤后点集</returns>
PointArray* applyVoxelGrid(Point3F points[], const int length, const float leafSize)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//加载点云
	for (int i = 0; i < length; i++)
	{
		const Point3F& point = points[i];
		PointXYZ pointXYZ = PointXYZ(point.X, point.Y, point.Z);
		sourceCloud->points.push_back(pointXYZ);
	}

	//创建降采样过滤器
	VoxelGrid<PointXYZ> voxelGrid;
	voxelGrid.setInputCloud(sourceCloud);
	voxelGrid.setLeafSize(leafSize, leafSize, leafSize);
	voxelGrid.filter(*targetCloud);

	const int outLength = targetCloud->size();
	PointArray* pointArray = new PointArray();
	pointArray->Length = outLength;
	pointArray->Points = new Point3F[outLength];
	for (int i = 0; i < outLength; i++)
	{
		const PointXYZ& pointXYZ = targetCloud->points[i];
		pointArray->Points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	return pointArray;
}

/// <summary>
/// 适用离群点移除
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="meanK">平均距离估计的最近邻居的数量</param>
/// <param name="stddevMult">标准差阈值系数</param>
/// <returns>过滤后点集</returns>
PointArray* applyOutlierRemoval(Point3F points[], const int length, const int meanK, const float stddevMult)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//加载点云
	for (int i = 0; i < length; i++)
	{
		const Point3F& point = points[i];
		PointXYZ pointXYZ = PointXYZ(point.X, point.Y, point.Z);
		sourceCloud->points.push_back(pointXYZ);
	}

	//初始化统计学离群点移除过滤器
	StatisticalOutlierRemoval<PointXYZ> statisticalOutlierRemoval;
	statisticalOutlierRemoval.setInputCloud(sourceCloud);
	statisticalOutlierRemoval.setMeanK(50);//设置平均距离估计的最近邻居的数量K	
	statisticalOutlierRemoval.setStddevMulThresh(1.0);//设置标准差阈值系数
	statisticalOutlierRemoval.filter(*targetCloud);	//执行过滤

	const int outLength = targetCloud->size();
	PointArray* pointArray = new PointArray();
	pointArray->Length = outLength;
	pointArray->Points = new Point3F[outLength];
	for (int i = 0; i < outLength; i++)
	{
		const PointXYZ& pointXYZ = targetCloud->points[i];
		pointArray->Points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	return pointArray;
}

/// <summary>
/// 释放资源
/// </summary>
/// <param name="pointer">指针</param>
void dispose(const PointArray* pointer)
{
	delete pointer;
}
