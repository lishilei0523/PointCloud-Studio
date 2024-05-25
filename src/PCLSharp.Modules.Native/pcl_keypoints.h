#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <point3fs.h>

/// <summary>
/// 检测NARF关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="angularResolution">角度分辨率</param>
/// <param name="maxAngleWidth">传感器水平边界角度</param>
/// <param name="maxAngleHeight">传感器垂直边界角度</param>
/// <param name="noiseLevel">所有与最近点的最大距离</param>
/// <param name="minRange">最小可见范围</param>
/// <param name="borderSize">边界尺寸</param>
/// <param name="supportSize">计算范围半径</param>
/// <returns>NARF关键点集</returns>
EXPORT_C Point3Fs* CALLING_MODE detectNARF(Point3F points[], int length, float angularResolution, float maxAngleWidth, float maxAngleHeight, float noiseLevel, float minRange, int borderSize, float supportSize);

/// <summary>
/// 检测ISS关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="salientRadius">显著半径</param>
/// <param name="nonMaxRadius">非极大值抑制半径</param>
/// <param name="threshold21">二一特征值比上限</param>
/// <param name="threshold32">三二特征值比上限</param>
/// <param name="minNeighborsCount">最小邻域点数</param>
/// <param name="threadsCount">线程数</param>
/// <returns>ISS关键点集</returns>
EXPORT_C Point3Fs* CALLING_MODE detectISS(Point3F points[], int length, float salientRadius, float nonMaxRadius, float threshold21, float threshold32, int minNeighborsCount, int threadsCount);

/// <summary>
/// 检测SIFT关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="minScale">尺度空间最小标准偏差</param>
/// <param name="octavesCount">金字塔组数量</param>
/// <param name="scalesPerOctaveCount">每组金字塔计算尺度</param>
/// <param name="minContrast">限制关键点检测阈值</param>
/// <returns>SIFT关键点集</returns>
EXPORT_C Point3Fs* CALLING_MODE detectSIFT(Point3F points[], int length, float minScale, int octavesCount, int scalesPerOctaveCount, float minContrast);

/// <summary>
/// 检测Harris关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="nonMaxSupression">非极大值抑制</param>
/// <param name="radius">搜索半径</param>
/// <param name="threshold">感兴趣阈值</param>
/// <returns>Harris关键点集</returns>
EXPORT_C Point3Fs* CALLING_MODE detectHarris(Point3F points[], int length, bool nonMaxSupression, float radius, float threshold);

/// <summary>
/// 检测SUSAN关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="nonMaxSupression">非极大值抑制</param>
/// <param name="radius">搜索半径</param>
/// <param name="distanceThreshold">距离阈值</param>
/// <param name="angularThreshold">角度阈值</param>
/// <param name="intensityThreshold">强度阈值</param>
/// <returns>SUSAN关键点集</returns>
EXPORT_C Point3Fs* CALLING_MODE detectSUSAN(Point3F points[], int length, bool nonMaxSupression, float radius, float distanceThreshold, float angularThreshold, float intensityThreshold);
