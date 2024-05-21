#pragma once
#define EXPORT extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <format>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <point3f.h>
#include <point3fs.h>
#include <primitives_map.h>

/// <summary>
/// 加载PCD文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT Point3Fs* CALLING_MODE loadPCD(const char* filePath);

/// <summary>
/// 加载PLY文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT Point3Fs* CALLING_MODE loadPLY(const char* filePath);

/// <summary>
/// 加载OBJ文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT Point3Fs* CALLING_MODE loadOBJ(const char* filePath);

/// <summary>
/// 保存PCD文本文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT void CALLING_MODE saveTextPCD(Point3F points[], int length, const char* filePath);

/// <summary>
/// 保存PCD二进制文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT void CALLING_MODE saveBinaryPCD(Point3F points[], int length, const char* filePath);

/// <summary>
/// 保存PLY文本文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT void CALLING_MODE saveTextPLY(Point3F points[], int length, const char* filePath);

/// <summary>
/// 保存PLY二进制文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT void CALLING_MODE saveBinaryPLY(Point3F points[], int length, const char* filePath);

/// <summary>
/// 释放资源
/// </summary>
/// <param name="pointer">指针</param>
EXPORT void CALLING_MODE dispose(const Point3Fs* pointer);
