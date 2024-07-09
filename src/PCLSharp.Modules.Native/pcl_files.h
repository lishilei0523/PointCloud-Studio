#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#elif __linux__
#define EXPORT_C extern "C"
#endif
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <point3fs.h>
#include <point3normal3.h>
#include <point3normal3s.h>
#include <point3color4.h>
#include <point3color4s.h>

/// <summary>
/// 加载PCD文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C Point3Fs* CALLING_MODE loadPCD(const char* filePath);

/// <summary>
/// 加载法向量PCD文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C Point3Normal3s* CALLING_MODE loadNormalPCD(const char* filePath);

/// <summary>
/// 加载颜色PCD文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C Point3Color4s* CALLING_MODE loadColorPCD(const char* filePath);

/// <summary>
/// 加载PLY文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C Point3Fs* CALLING_MODE loadPLY(const char* filePath);

/// <summary>
/// 加载法向量PLY文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C Point3Normal3s* CALLING_MODE loadNormalPLY(const char* filePath);

/// <summary>
/// 加载颜色PLY文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C Point3Color4s* CALLING_MODE loadColorPLY(const char* filePath);

/// <summary>
/// 加载OBJ文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C Point3Fs* CALLING_MODE loadOBJ(const char* filePath);

/// <summary>
/// 加载法向量OBJ文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C Point3Normal3s* CALLING_MODE loadNormalOBJ(const char* filePath);

/// <summary>
/// 加载颜色OBJ文件
/// </summary>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C Point3Color4s* CALLING_MODE loadColorOBJ(const char* filePath);

/// <summary>
/// 保存PCD文本文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveTextPCD(Point3F points[], int length, const char* filePath);

/// <summary>
/// 保存PCD二进制文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveBinaryPCD(Point3F points[], int length, const char* filePath);

/// <summary>
/// 保存法向量PCD文本文件
/// </summary>
/// <param name="pointNormals">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveNormalTextPCD(Point3Normal3 pointNormals[], int length, const char* filePath);

/// <summary>
/// 保存法向量PCD二进制文件
/// </summary>
/// <param name="pointNormals">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveNormalBinaryPCD(Point3Normal3 pointNormals[], int length, const char* filePath);

/// <summary>
/// 保存颜色PCD文本文件
/// </summary>
/// <param name="pointColors">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveColorTextPCD(Point3Color4 pointColors[], int length, const char* filePath);

/// <summary>
/// 保存颜色PCD二进制文件
/// </summary>
/// <param name="pointColors">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveColorBinaryPCD(Point3Color4 pointColors[], int length, const char* filePath);

/// <summary>
/// 保存PLY文本文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveTextPLY(Point3F points[], int length, const char* filePath);

/// <summary>
/// 保存PLY二进制文件
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveBinaryPLY(Point3F points[], int length, const char* filePath);

/// <summary>
/// 保存法向量PLY文本文件
/// </summary>
/// <param name="pointNormals">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveNormalTextPLY(Point3Normal3 pointNormals[], int length, const char* filePath);

/// <summary>
/// 保存法向量PLY二进制文件
/// </summary>
/// <param name="pointNormals">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveNormalBinaryPLY(Point3Normal3 pointNormals[], int length, const char* filePath);

/// <summary>
/// 保存颜色PLY文本文件
/// </summary>
/// <param name="pointColors">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveColorTextPLY(Point3Color4 pointColors[], int length, const char* filePath);

/// <summary>
/// 保存颜色PLY二进制文件
/// </summary>
/// <param name="pointColors">点集</param>
/// <param name="length">点集长度</param>
/// <param name="filePath">文件路径</param>
/// <returns>点集</returns>
EXPORT_C void CALLING_MODE saveColorBinaryPLY(Point3Color4 pointColors[], int length, const char* filePath);
