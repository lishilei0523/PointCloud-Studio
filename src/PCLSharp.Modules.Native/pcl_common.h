#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include "point3f.h"
#include "point3fs.h"
#include "pose.h"

/// <summary>
/// 估算质心
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <returns>质心坐标点</returns>
EXPORT_C Point3F* CALLING_MODE estimateCentroid(Point3F points[], int length);

/// <summary>
/// 仿射变换
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="pose">位姿</param>
/// <returns>变换后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE affineTransform(Point3F points[], int length, Pose pose);
