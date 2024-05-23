#pragma once

/// <summary>
/// 坐标点颜色
/// </summary>
struct Point3Color4
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Point3Color4() = default;

	/// <summary>
	/// 创建坐标点颜色构造器
	/// </summary>
	/// <param name="x">X坐标</param>
	/// <param name="y">Y坐标</param>
	/// <param name="z">Z坐标</param>
	/// <param name="r">R值</param>
	/// <param name="g">G值</param>
	/// <param name="b">B值</param>
	/// <param name="a">A值</param>
	Point3Color4(const float& x, const float& y, const float& z, const unsigned char& r, const unsigned char& g, const unsigned char& b, const unsigned char& a)
		:X(x), Y(y), Z(z), R(r), G(g), B(b), A(a == 0 ? 255 : a)
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
	/// R值
	/// </summary>
	unsigned char R;

	/// <summary>
	/// G值
	/// </summary>
	unsigned char G;

	/// <summary>
	/// B值
	/// </summary>
	unsigned char B;

	/// <summary>
	/// A值
	/// </summary>
	unsigned char A;
};
