#include "dispose.h"

/// <summary>
/// 释放坐标点
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3F(const Point3F* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点集
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Fs(const Point3Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放法向量
/// </summary>
/// <param name="pointer">指针</param>
void disposeNormal3F(const Normal3F* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放法向量集
/// </summary>
/// <param name="pointer">指针</param>
void disposeNormal3Fs(const Normal3Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点法向量
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Normal3(const Point3Normal3* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点法向量集
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Normal3s(const Point3Normal3s* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点颜色
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Color4(const Point3Color4* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点颜色集
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Color4s(const Point3Color4s* pointer)
{
	delete pointer;
}
