#pragma once
#define EXPORT_CPP extern __declspec(dllexport)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "point3f.h"
#include "point3fs.h"
#include "normal3f.h"
#include "normal3fs.h"

namespace pclsharp
{
	/// <summary>
	/// 坐标点映射PointXYZ
	/// </summary>
	/// <param name="point3F">坐标点</param>
	/// <returns>PointXYZ</returns>
	EXPORT_CPP pcl::PointXYZ toPointXYZ(const Point3F& point3F);

	/// <summary>
	/// 坐标点集映射点云
	/// </summary>
	/// <param name="point3Fs">坐标点集</param>
	/// <param name="length">长度</param>
	/// <returns>点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PointXYZ> toPointCloud(Point3F point3Fs[], const int& length);

	/// <summary>
	/// 点云映射坐标点集结构体
	/// </summary>
	/// <param name="pointCloud">点云</param>
	/// <returns>坐标点集结构体</returns>
	EXPORT_CPP Point3Fs* toPoint3Fs(const pcl::PointCloud<pcl::PointXYZ>& pointCloud);

	/// <summary>
	/// 法向量映射Normal
	/// </summary>
	/// <param name="normal3F">法向量</param>
	/// <returns>Normal</returns>
	EXPORT_CPP pcl::Normal toNormal(const Normal3F& normal3F);

	/// <summary>
	/// 法向量集映射点云
	/// </summary>
	/// <param name="normal3Fs">法向量集</param>
	/// <param name="length">长度</param>
	/// <returns>点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::Normal> toPointCloud(Normal3F normal3Fs[], const int& length);

	/// <summary>
	/// 点云映射法向量集结构体
	/// </summary>
	/// <param name="pointCloud">点云</param>
	/// <returns>法向量集结构体</returns>
	EXPORT_CPP Normal3Fs* toNormal3Fs(const pcl::PointCloud<pcl::Normal>& pointCloud);
}
