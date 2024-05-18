#pragma once
#include "point3f.h"

/// <summary>
/// 3D坐标点数组
/// </summary>
struct PointArray
{
public:

	/// <summary>
	/// 析构函数
	/// </summary>
	~PointArray()
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
