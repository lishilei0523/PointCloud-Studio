#pragma once
#include "point3color4.h"

/// <summary>
/// 坐标点颜色集
/// </summary>
struct Point3Color4s
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Point3Color4s() = default;

	/// <summary>
	/// 创建坐标点颜色集构造器
	/// </summary>
	/// <param name="pointColors">坐标点颜色集指针</param>
	/// <param name="length">长度</param>
	Point3Color4s(Point3Color4* pointColors, const int& length)
		:PointColors(pointColors), Length(length)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~Point3Color4s()
	{
		delete[] this->PointColors;
	}

	/// <summary>
	/// 坐标点颜色集指针
	/// </summary>
	Point3Color4* PointColors;

	/// <summary>
	/// 长度
	/// </summary>
	int Length;
};
