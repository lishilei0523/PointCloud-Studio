#pragma once

/// <summary>
/// 坐标点
/// </summary>
struct Point3F
{
public:

	/// <summary>
	/// 无参构造器
	/// </summary>
	Point3F() = default;

	/// <summary>
	/// 创建坐标点构造器
	/// </summary>
	/// <param name="x">X坐标</param>
	/// <param name="y">Y坐标</param>
	/// <param name="z">Z坐标</param>
	Point3F(const float x, const float y, const float z)
		:X(x), Y(y), Z(z)
	{

	}

	/// <summary>
	/// X坐标
	/// </summary>
	float X;

	/// <summary>
	/// Y坐标
	/// </summary>
	float Y;

	/// <summary>
	/// Z坐标
	/// </summary>
	float Z;
};
