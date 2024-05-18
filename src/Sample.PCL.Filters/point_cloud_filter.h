#pragma once
#define EXPORT extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "point3f.h"
#include "point_array.h"

/// <summary>
/// 适用直通滤波
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="axis">过滤坐标轴</param>
/// <param name="limitMin">过滤范围最小值</param>
/// <param name="limixMax">过滤范围最大值</param>
/// <returns>过滤后点集</returns>
EXPORT PointArray* CALLING_MODE applyPassThrogh(Point3F points[], int length, const char* axis, float limitMin, float limixMax);

/// <summary>
/// 适用均匀采样
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="radius">采样半径</param>
/// <returns>过滤后点集</returns>
EXPORT PointArray* CALLING_MODE applyUniformSampling(Point3F points[], int length, float radius);

/// <summary>
/// 适用体素降采样
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="leafSize">叶尺寸</param>
/// <returns>过滤后点集</returns>
EXPORT PointArray* CALLING_MODE applyVoxelGrid(Point3F points[], int length, float leafSize);

/// <summary>
/// 适用离群点移除
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="meanK">平均距离估计的最近邻居的数量</param>
/// <param name="stddevMult">标准差阈值系数</param>
/// <returns>过滤后点集</returns>
EXPORT PointArray* CALLING_MODE applyOutlierRemoval(Point3F points[], int length, int meanK, float stddevMult);

/// <summary>
/// 释放资源
/// </summary>
/// <param name="pointer">指针</param>
EXPORT void CALLING_MODE dispose(const PointArray* pointer);
