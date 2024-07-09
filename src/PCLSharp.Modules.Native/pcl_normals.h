#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#elif __linux__
#define EXPORT_C extern "C"
#define CALLING_MODE __attribute__((__stdcall__))
#endif
#include <point3f.h>
#include <normal3fs.h>

/// <summary>
/// 估算法向量
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="k">搜索近邻数量</param>
/// <returns>法向量集</returns>
EXPORT_C Normal3Fs* CALLING_MODE estimateNormalsByK(Point3F points[], int length, int k);

/// <summary>
/// 估算法向量
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="radius">搜索半径</param>
/// <returns>法向量集</returns>
EXPORT_C Normal3Fs* CALLING_MODE estimateNormalsByRadius(Point3F points[], int length, float radius);

/// <summary>
/// 估算法向量 (OMP)
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="k">搜索近邻数量</param>
/// <returns>法向量集</returns>
EXPORT_C Normal3Fs* CALLING_MODE estimateNormalsByKP(Point3F points[], int length, int k);

/// <summary>
/// 估算法向量 (OMP)
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="radius">搜索半径</param>
/// <returns>法向量集</returns>
EXPORT_C Normal3Fs* CALLING_MODE estimateNormalsByRadiusP(Point3F points[], int length, float radius);
