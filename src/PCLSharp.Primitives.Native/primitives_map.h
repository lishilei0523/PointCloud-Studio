#pragma once
#define EXPORT_CPP extern __declspec(dllexport)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "point3f.h"
#include "point3fs.h"
#include "point3color4.h"
#include "point3color4s.h"
#include "point3normal3.h"
#include "point3normal3s.h"
#include "normal3f.h"
#include "normal3fs.h"

namespace pclsharp
{
	/// <summary>
	/// 坐标点集映射点云
	/// </summary>
	/// <param name="point3Fs">坐标点集</param>
	/// <param name="length">长度</param>
	/// <returns>PointXYZ点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PointXYZ>::Ptr toPointCloud(Point3F point3Fs[], const int& length);

	/// <summary>
	/// 点云映射坐标点集
	/// </summary>
	/// <param name="pointCloud">PointXYZ点云</param>
	/// <returns>坐标点集</returns>
	EXPORT_CPP Point3Fs* toPoint3Fs(const pcl::PointCloud<pcl::PointXYZ>& pointCloud);

	/// <summary>
	/// 法向量集映射点云
	/// </summary>
	/// <param name="normal3Fs">法向量集</param>
	/// <param name="length">长度</param>
	/// <returns>Normal点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::Normal>::Ptr toPointCloud(Normal3F normal3Fs[], const int& length);

	/// <summary>
	/// 点云映射法向量集
	/// </summary>
	/// <param name="pointCloud">Normal点云</param>
	/// <returns>法向量集</returns>
	EXPORT_CPP Normal3Fs* toNormal3Fs(const pcl::PointCloud<pcl::Normal>& pointCloud);

	/// <summary>
	/// 坐标点法向量集映射点云
	/// </summary>
	/// <param name="point3Normal3s">坐标点法向量集</param>
	/// <param name="length">长度</param>
	/// <returns>PointNormal点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PointNormal>::Ptr toPointCloud(Point3Normal3 point3Normal3s[], const int& length);

	/// <summary>
	/// 点云映射坐标点法向量集
	/// </summary>
	/// <param name="pointCloud">PointNormal点云</param>
	/// <returns>坐标点法向量集</returns>
	EXPORT_CPP Point3Normal3s* toPoint3Normal3s(const pcl::PointCloud<pcl::PointNormal>& pointCloud);

	/// <summary>
	/// 坐标点颜色集映射点云
	/// </summary>
	/// <param name="point3Color4s">坐标点颜色集</param>
	/// <param name="length">长度</param>
	/// <returns>PointXYZRGBA点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PointXYZRGBA>::Ptr toPointCloud(Point3Color4 point3Color4s[], const int& length);

	/// <summary>
	/// 点云映射坐标点颜色集
	/// </summary>
	/// <param name="pointCloud">PointXYZRGBA点云</param>
	/// <returns>坐标点颜色集</returns>
	EXPORT_CPP Point3Color4s* toPoint3Color4s(const pcl::PointCloud<pcl::PointXYZRGBA>& pointCloud);
}
