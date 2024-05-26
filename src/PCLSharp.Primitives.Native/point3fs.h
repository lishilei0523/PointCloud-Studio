#pragma once
#include "point3f.h"

/// <summary>
/// 坐标点集
/// </summary>
struct Point3Fs
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Point3Fs() = default;

	/// <summary>
	/// 创建坐标点集构造器
	/// </summary>
	/// <param name="points">点集指针</param>
	/// <param name="length">长度</param>
	Point3Fs(Point3F* points, const int& length)
		:Points(points), Length(length)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~Point3Fs()
	{
		delete[] this->Points;
	}

	/// <summary>
	/// 点集指针
	/// </summary>
	Point3F* Points;

	/// <summary>
	/// 长度
	/// </summary>
	int Length;
};
