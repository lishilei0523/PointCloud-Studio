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

/// <summary>
/// 长方体剪裁
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="minPoint">最小坐标点</param>
/// <param name="maxPoint">最大坐标点</param>
/// <param name="negative">true: 剪裁/false: 保留</param>
/// <returns>剪裁后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE cropBox(Point3F points[], int length, Point3F minPoint, Point3F maxPoint, bool negative);

/// <summary>
/// 凸包剪裁
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="contourPoints">轮廓点集</param>
/// <param name="contourLength">轮廓点集长度</param>
/// <param name="dimensionsCount">维度数</param>
/// <returns>剪裁后点云</returns>
EXPORT_C Point3Fs* CALLING_MODE cropConvexHull(Point3F points[], int length, Point3F contourPoints[], int contourLength, int dimensionsCount);

/// <summary>
/// 投射平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="a">平面方程系数a</param>
/// <param name="b">平面方程系数b</param>
/// <param name="c">平面方程系数c</param>
/// <param name="d">平面方程系数d</param>
/// <returns>投射后点云</returns>
/// <remarks>平面方程: ax + by +cz + d = 0</remarks>
EXPORT_C Point3Fs* CALLING_MODE projectPlane(Point3F points[], int length, float a, float b, float c, float d);

/// <summary>
/// 提取边框
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <returns>边框点云</returns>
EXPORT_C Point3Fs* CALLING_MODE extractBorder(Point3F points[], int length);

/// <summary>
/// 提取边界
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="featureRadius">特征半径</param>
/// <param name="angleThreshold">角度阈值</param>
/// <param name="threadsCount">线程数</param>
/// <returns>边界点云</returns>
EXPORT_C Point3Fs* CALLING_MODE extractBoundary(Point3F points[], int length, int normalK, float featureRadius, float angleThreshold, int threadsCount);
