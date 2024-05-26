#pragma once

/// <summary>
/// 坐标点法向量
/// </summary>
struct Point3Normal3
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Point3Normal3() = default;

	/// <summary>
	/// 创建坐标点法向量构造器
	/// </summary>
	/// <param name="x">X坐标</param>
	/// <param name="y">Y坐标</param>
	/// <param name="z">Z坐标</param>
	/// <param name="nx">法向量X坐标</param>
	/// <param name="ny">法向量Y坐标</param>
	/// <param name="nz">法向量Z坐标</param>
	Point3Normal3(const float& x, const float& y, const float& z, const float& nx, const float& ny, const float& nz)
		:X(x), Y(y), Z(z), NX(nx), NY(ny), NZ(nz)
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

	/// <summary>
	/// 法向量X坐标
	/// </summary>
	float NX;

	/// <summary>
	/// 法向量Y坐标
	/// </summary>
	float NY;

	/// <summary>
	/// 法向量Z坐标
	/// </summary>
	float NZ;
};
