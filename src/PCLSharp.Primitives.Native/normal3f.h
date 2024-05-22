#pragma once

/// <summary>
/// 法向量
/// </summary>
struct Normal3F
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Normal3F() = default;

	/// <summary>
	/// 创建法向量构造器
	/// </summary>
	/// <param name="nx">法向量X坐标</param>
	/// <param name="ny">法向量Y坐标</param>
	/// <param name="nz">法向量Z坐标</param>
	Normal3F(const float& nx, const float& ny, const float& nz)
		:NX(nx), NY(ny), NZ(nz)
	{

	}

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
