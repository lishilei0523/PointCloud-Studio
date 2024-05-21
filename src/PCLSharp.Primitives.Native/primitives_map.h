#pragma once
#define EXPORT extern __declspec(dllexport)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "point3f.h"
#include "point3fs.h"

namespace pclsharp
{
	/// <summary>
	/// 坐标点映射PointXYZ
	/// </summary>
	/// <param name="point3F">3D坐标点</param>
	/// <returns>PointXYZ</returns>
	EXPORT pcl::PointXYZ toPointXYZ(const Point3F& point3F);

	/// <summary>
	/// 坐标点集映射点云
	/// </summary>
	/// <param name="point3Fs">坐标点集</param>
	/// <param name="length">长度</param>
	/// <returns>点云</returns>
	EXPORT pcl::PointCloud<pcl::PointXYZ> toPointCloud(Point3F point3Fs[], const int& length);

	/// <summary>
	/// 点云映射坐标点集结构体
	/// </summary>
	/// <param name="pointCloud">点云</param>
	/// <returns>坐标点集结构体</returns>
	EXPORT Point3Fs* toPoint3Fs(const pcl::PointCloud<pcl::PointXYZ>& pointCloud);
}
