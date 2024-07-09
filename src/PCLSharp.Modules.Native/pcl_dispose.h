#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#elif __linux__
#define EXPORT_C extern "C"
#define CALLING_MODE __attribute__((__cdecl__))
#endif
#include <point3f.h>
#include <point3fs.h>
#include <normal3f.h>
#include <normal3fs.h>
#include <point3normal3.h>
#include <point3normal3s.h>
#include <point3color4.h>
#include <point3color4s.h>
#include <narf36f.h>
#include <narf36fs.h>
#include <pfh_signature125f.h>
#include <pfh_signature125fs.h>
#include <fpfh_signature33f.h>
#include <fpfh_signature33fs.h>
#include <shape_context1980f.h>
#include <shape_context1980fs.h>
#include <shot352f.h>
#include <shot352fs.h>
#include <alignment_result.h>

/// <summary>
/// 释放坐标点
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposePoint3F(const Point3F* pointer);

/// <summary>
/// 释放坐标点集
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposePoint3Fs(const Point3Fs* pointer);

/// <summary>
/// 释放坐标点集分组
/// </summary>
/// <param name="pointer">二级指针</param>
/// <param name="groupCount">分组数</param>
EXPORT_C void CALLING_MODE disposePoint3FsGroup(const Point3Fs** pointer, int groupCount);

/// <summary>
/// 释放法向量
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeNormal3F(const Normal3F* pointer);

/// <summary>
/// 释放法向量集
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeNormal3Fs(const Normal3Fs* pointer);

/// <summary>
/// 释放法向量集分组
/// </summary>
/// <param name="pointer">二级指针</param>
/// <param name="groupCount">分组数</param>
EXPORT_C void CALLING_MODE disposeNormal3FsGroup(const Point3Fs** pointer, int groupCount);

/// <summary>
/// 释放坐标点法向量
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposePoint3Normal3(const Point3Normal3* pointer);

/// <summary>
/// 释放坐标点法向量集
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposePoint3Normal3s(const Point3Normal3s* pointer);

/// <summary>
/// 释放坐标点法向量集分组
/// </summary>
/// <param name="pointer">二级指针</param>
/// <param name="groupCount">分组数</param>
EXPORT_C void CALLING_MODE disposePoint3Normal3sGroup(const Point3Fs** pointer, int groupCount);

/// <summary>
/// 释放坐标点颜色
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposePoint3Color4(const Point3Color4* pointer);

/// <summary>
/// 释放坐标点颜色集
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposePoint3Color4s(const Point3Color4s* pointer);

/// <summary>
/// 释放坐标点颜色集分组
/// </summary>
/// <param name="pointer">二级指针</param>
/// <param name="groupCount">分组数</param>
EXPORT_C void CALLING_MODE disposePoint3Color4sGroup(const Point3Fs** pointer, int groupCount);

/// <summary>
/// 释放NARF描述子
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeNarf36F(const Narf36F* pointer);

/// <summary>
/// 释放NARF描述子集
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeNarf36Fs(const Narf36Fs* pointer);

/// <summary>
/// 释放PFH描述子
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposePFHSignature125F(const PFHSignature125F* pointer);

/// <summary>
/// 释放PFH描述子集
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposePFHSignature125Fs(const PFHSignature125Fs* pointer);

/// <summary>
/// 释放FPFH描述子
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeFPFHSignature33F(const FPFHSignature33F* pointer);

/// <summary>
/// 释放FPFH描述子集
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeFPFHSignature33Fs(const FPFHSignature33Fs* pointer);

/// <summary>
/// 释放3DSC描述子
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeShapeContext1980F(const ShapeContext1980F* pointer);

/// <summary>
/// 释放3DSC描述子集
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeShapeContext1980Fs(const ShapeContext1980Fs* pointer);

/// <summary>
/// 释放SHOT描述子
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeShot352F(const Shot352F* pointer);

/// <summary>
/// 释放SHOT描述子集
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeShot352Fs(const Shot352Fs* pointer);

/// <summary>
/// 释放配准结果
/// </summary>
/// <param name="pointer">指针</param>
EXPORT_C void CALLING_MODE disposeAlignmentResult(const AlignmentResult* pointer);
