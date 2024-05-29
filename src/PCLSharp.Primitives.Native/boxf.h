#pragma once
#include "point3f.h"

/// <summary>
/// 长方体
/// </summary>
struct BoxF
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	BoxF() = default;

	/// <summary>
	/// 创建长方体构造器
	/// </summary>
	/// <param name="minPoint">最小坐标点</param>
	/// <param name="maxPoint">最大坐标点</param>
	BoxF(const Point3F& minPoint, const Point3F& maxPoint)
		:MinPoint(minPoint), MaxPoint(maxPoint)
	{

	}

	/// <summary>
	/// 最小坐标点
	/// </summary>
	Point3F MinPoint;

	/// <summary>
	/// 最大坐标点
	/// </summary>
	Point3F MaxPoint;
};
