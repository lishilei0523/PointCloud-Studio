#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <point3fs.h>
#include <point3color4.h>
#include <point3color4s.h>

/// <summary>
/// 分割平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="optimizeCoefficients">是否优化模型系数</param>
/// <param name="probability">概率</param>
/// <param name="distanceThreshold">距离阈值</param>
/// <param name="maxIterationsCount">最大迭代次数</param>
/// <returns>平面点云</returns>
EXPORT_C Point3Fs* CALLING_MODE segmentPlane(Point3F points[], int length, bool optimizeCoefficients, float probability, float distanceThreshold, int maxIterationsCount);

/// <summary>
/// 分割球体
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="optimizeCoefficients">是否优化模型系数</param>
/// <param name="probability">概率</param>
/// <param name="distanceThreshold">距离阈值</param>
/// <param name="minRadius">球体最小半径</param>
/// <param name="maxRadius">球体最大半径</param>
/// <param name="maxIterationsCount">最大迭代次数</param>
/// <returns>球体点云</returns>
EXPORT_C Point3Fs* CALLING_MODE segmentSphere(Point3F points[], int length, bool optimizeCoefficients, float probability, float distanceThreshold, float minRadius, float maxRadius, int maxIterationsCount);

/// <summary>
/// 欧几里得聚类分割
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="clusterTolerance">簇搜索容差</param>
/// <param name="minClusterSize">簇最小尺寸</param>
/// <param name="maxClusterSize">簇最大尺寸</param>
/// <param name="clustersCount">点云簇数</param>
/// <returns>点云簇列表</returns>
EXPORT_C Point3Fs** CALLING_MODE euclidClusterSegment(Point3F points[], int length, float clusterTolerance, int minClusterSize, int maxClusterSize, int& clustersCount);

/// <summary>
/// 区域生长分割
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="clusterK">簇K</param>
/// <param name="smoothnessThreshold">平滑阈值（角度）</param>
/// <param name="curvatureThreshold">曲率阈值</param>
/// <param name="minClusterSize">簇最小尺寸</param>
/// <param name="maxClusterSize">簇最大尺寸</param>
/// <param name="threadsCount">线程数</param>
/// <param name="clustersCount">点云簇数</param>
/// <returns>点云簇列表</returns>
EXPORT_C Point3Fs** CALLING_MODE regionGrowingSegment(Point3F points[], int length, int normalK, int clusterK, float smoothnessThreshold, float curvatureThreshold, int minClusterSize, int maxClusterSize, int threadsCount, int& clustersCount);

/// <summary>
/// 区域生长颜色分割
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="clusterK">簇K</param>
/// <param name="distanceThreshold">距离阈值</param>
/// <param name="smoothnessThreshold">平滑阈值（角度）</param>
/// <param name="curvatureThreshold">曲率阈值</param>
/// <param name="pointColorThreshold">点颜色阈值</param>
/// <param name="regionColorThreshold">区域颜色阈值</param>
/// <param name="minClusterSize">簇最小尺寸</param>
/// <param name="maxClusterSize">簇最大尺寸</param>
/// <param name="threadsCount">线程数</param>
/// <param name="clustersCount">点云簇数</param>
/// <returns>点云簇列表</returns>
EXPORT_C Point3Color4s** CALLING_MODE regionGrowingColorSegment(Point3Color4 points[], int length, int normalK, int clusterK, float distanceThreshold, float smoothnessThreshold, float curvatureThreshold, float pointColorThreshold, float regionColorThreshold, int minClusterSize, int maxClusterSize, int threadsCount, int& clustersCount);
