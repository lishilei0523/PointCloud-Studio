#include <format>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <primitives_map.h>
#include "pcl_files.h"
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
	throw invalid_argument(message.c_str());
}

/// <summary>
/// 加载法向量PCD文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Normal3s* loadNormalPCD(const char* filePath)
{
	PointCloud<PointNormal> cloud;
	const int& loadStatus = pcl::io::loadPCDFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Normal3s* point3Normal3s = pclsharp::toPoint3Normal3s(cloud);
		return point3Normal3s;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// 加载颜色PCD文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Color4s* loadColorPCD(const char* filePath)
{
	PointCloud<PointXYZRGBA> cloud;
	const int& loadStatus = pcl::io::loadPCDFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Color4s* point3Color4s = pclsharp::toPoint3Color4s(cloud);
		return point3Color4s;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// 加载PLY文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Fs* loadPLY(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadPLYFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// 加载法向量PLY文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Normal3s* loadNormalPLY(const char* filePath)
{
	PointCloud<PointNormal> cloud;
	const int& loadStatus = pcl::io::loadPLYFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Normal3s* point3Normal3s = pclsharp::toPoint3Normal3s(cloud);
		return point3Normal3s;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// 加载颜色PLY文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Color4s* loadColorPLY(const char* filePath)
{
	PointCloud<PointXYZRGBA> cloud;
	const int& loadStatus = pcl::io::loadPLYFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Color4s* point3Color4s = pclsharp::toPoint3Color4s(cloud);
		return point3Color4s;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// 加载OBJ文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Fs* loadOBJ(const char* filePath)
{
	PointCloud<PointXYZ> cloud;
	const int& loadStatus = pcl::io::loadOBJFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Fs* point3Fs = pclsharp::toPoint3Fs(cloud);
		return point3Fs;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// 加载法向量OBJ文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Normal3s* loadNormalOBJ(const char* filePath)
{
	PointCloud<PointNormal> cloud;
	const int& loadStatus = pcl::io::loadOBJFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Normal3s* point3Normal3s = pclsharp::toPoint3Normal3s(cloud);
		return point3Normal3s;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw invalid_argument(message.c_str());
}

/// <summary>
/// 加载颜色OBJ文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
Point3Color4s* loadColorOBJ(const char* filePath)
{
	PointCloud<PointXYZRGBA> cloud;
	const int& loadStatus = pcl::io::loadOBJFile(filePath, cloud);
	if (loadStatus == 0)
	{
		Point3Color4s* point3Color4s = pclsharp::toPoint3Color4s(cloud);
		return point3Color4s;
	}

	const string message = std::format("加载\"{}\"时出错！", filePath);
	throw invalid_argument(message.c_str());
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
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePCDFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
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
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePCDFileBinaryCompressed(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// 保存法向量PCD文本文件
/// </summary>
/// <param name="pointNormals">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveNormalTextPCD(Point3Normal3 pointNormals[], int length, const char* filePath)
{
	const PointCloud<PointNormal>::Ptr& cloud = pclsharp::toPointCloud(pointNormals, length);
	const int& saveStatus = pcl::io::savePCDFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// 保存法向量PCD二进制文件
/// </summary>
/// <param name="pointNormals">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveNormalBinaryPCD(Point3Normal3 pointNormals[], int length, const char* filePath)
{
	const PointCloud<PointNormal>::Ptr& cloud = pclsharp::toPointCloud(pointNormals, length);
	const int& saveStatus = pcl::io::savePCDFileBinaryCompressed(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// 保存颜色PCD文本文件
/// </summary>
/// <param name="pointColors">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveColorTextPCD(Point3Color4 pointColors[], int length, const char* filePath)
{
	const PointCloud<PointXYZRGBA>::Ptr& cloud = pclsharp::toPointCloudRGBA(pointColors, length);
	const int& saveStatus = pcl::io::savePCDFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// 保存颜色PCD二进制文件
/// </summary>
/// <param name="pointColors">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveColorBinaryPCD(Point3Color4 pointColors[], int length, const char* filePath)
{
	const PointCloud<PointXYZRGBA>::Ptr& cloud = pclsharp::toPointCloudRGBA(pointColors, length);
	const int& saveStatus = pcl::io::savePCDFileBinaryCompressed(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
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
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePLYFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
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
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const int& saveStatus = pcl::io::savePLYFileBinary(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// 保存法向量PLY文本文件
/// </summary>
/// <param name="pointNormals">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveNormalTextPLY(Point3Normal3 pointNormals[], int length, const char* filePath)
{
	const PointCloud<PointNormal>::Ptr& cloud = pclsharp::toPointCloud(pointNormals, length);
	const int& saveStatus = pcl::io::savePLYFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// 保存法向量PLY二进制文件
/// </summary>
/// <param name="pointNormals">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveNormalBinaryPLY(Point3Normal3 pointNormals[], int length, const char* filePath)
{
	const PointCloud<PointNormal>::Ptr& cloud = pclsharp::toPointCloud(pointNormals, length);
	const int& saveStatus = pcl::io::savePLYFileBinary(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// 保存颜色PLY文本文件
/// </summary>
/// <param name="pointColors">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveColorTextPLY(Point3Color4 pointColors[], int length, const char* filePath)
{
	const PointCloud<PointXYZRGBA>::Ptr& cloud = pclsharp::toPointCloudRGBA(pointColors, length);
	const int& saveStatus = pcl::io::savePLYFileASCII(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
	}
}

/// <summary>
/// 保存颜色PLY二进制文件
/// </summary>
/// <param name="pointColors">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
void saveColorBinaryPLY(Point3Color4 pointColors[], int length, const char* filePath)
{
	const PointCloud<PointXYZRGBA>::Ptr& cloud = pclsharp::toPointCloudRGBA(pointColors, length);
	const int& saveStatus = pcl::io::savePLYFileBinary(filePath, *cloud);
	if (saveStatus != 0)
	{
		const string message = std::format("保存\"{}\"时出错！", filePath);
		throw invalid_argument(message.c_str());
	}
}
