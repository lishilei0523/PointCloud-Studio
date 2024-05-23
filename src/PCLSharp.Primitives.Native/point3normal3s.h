#pragma once
#include "point3normal3.h"

/// <summary>
/// 坐标点法向量集结构体
/// </summary>
struct Point3Normal3s
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Point3Normal3s() = default;

	/// <summary>
	/// 创建坐标点法向量集结构体构造器
	/// </summary>
	/// <param name="pointNormals">坐标点法向量集指针</param>
	/// <param name="length">长度</param>
	Point3Normal3s(Point3Normal3* pointNormals, const int& length)
		:PointNormals(pointNormals), Length(length)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~Point3Normal3s()
	{
		delete[] this->PointNormals;
	}

	/// <summary>
	/// 坐标点法向量集指针
	/// </summary>
	Point3Normal3* PointNormals;

	/// <summary>
	/// 长度
	/// </summary>
	int Length;
};
