#include <format>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <primitives_map.h>
#include "pcl_conductor.h"
using namespace std;
using namespace pcl;

/// <summary>
/// 加载PCD文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Fs* loadPCD(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadPCDFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw exception(message.c_str());
}

/// <summary>
/// 加载PLY文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Fs* CALLING_MODE loadPLY(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadPLYFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw exception(message.c_str());
}

/// <summary>
/// 加载OBJ文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Fs* CALLING_MODE loadOBJ(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadOBJFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw exception(message.c_str());
}

/// <summary>
/// 保存PCD文本文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveTextPCD(Point3F points[], int length, const char* filePath)
{
	const PointCloud<PointXYZ>& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePCDFileASCII(filePath, cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw exception(message.c_str());
	}
}

/// <summary>
/// 保存PCD二进制文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveBinaryPCD(Point3F points[], int length, const char* filePath)
{
	const PointCloud<PointXYZ>& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePCDFileBinaryCompressed(filePath, cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw exception(message.c_str());
	}
}

/// <summary>
/// 保存PLY文本文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveTextPLY(Point3F points[], int length, const char* filePath)
{
	const PointCloud<PointXYZ>& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePLYFileASCII(filePath, cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw exception(message.c_str());
	}
}

/// <summary>
/// 保存PLY二进制文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveBinaryPLY(Point3F points[], int length, const char* filePath)
{
	const PointCloud<PointXYZ>& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePLYFileBinary(filePath, cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw exception(message.c_str());
	}
}

/// <summary>
/// 释放资源
/// </summary>
/// <param name="pointer">指针</param>
void dispose(const Point3Fs* pointer)
{
	delete pointer;
}
