#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#elif __linux__
#define EXPORT_C extern "C"
#define CALLING_MODE __attribute__((__stdcall__))
#endif
#include <point3f.h>
#include <point3fs.h>

/// <summary>
/// K近邻搜索
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="referencePoint">参考坐标点</param>
/// <param name="k">近邻数量</param>
/// <returns>结果点集</returns>
EXPORT_C Point3Fs* CALLING_MODE kSearch(Point3F points[], int length, Point3F referencePoint, int k);

/// <summary>
/// 半径搜索
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="referencePoint">参考坐标点</param>
/// <param name="radius">搜索半径</param>
/// <returns>结果点集</returns>
EXPORT_C Point3Fs* CALLING_MODE radiusSearch(Point3F points[], int length, Point3F referencePoint, float radius);

/// <summary>
/// 八叉树搜索
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="referencePoint">参考坐标点</param>
/// <param name="resolution">分辨率</param>
/// <returns>结果点集</returns>
EXPORT_C Point3Fs* CALLING_MODE octreeSearch(Point3F points[], int length, Point3F referencePoint, float resolution);
