#pragma once

/// <summary>
/// NARF描述子
/// </summary>
struct Narf36F
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Narf36F() = default;

	/// <summary>
	/// 创建NARF描述子构造器
	/// </summary>
	/// <param name="x">X坐标</param>
	/// <param name="y">Y坐标</param>
	/// <param name="z">Z坐标</param>
	/// <param name="pitch">俯仰角(RX)</param>
	/// <param name="yaw">偏航角(RY)</param>
	/// <param name="roll">翻滚角(RZ)</param>
	Narf36F(const float& x, const float& y, const float& z, const float& pitch, const float& yaw, const float& roll)
		:X(x), Y(y), Z(z), Pitch(pitch), Yaw(yaw), Roll(roll), Features{ 0.0f }
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
	/// 俯仰角(RX)
	/// </summary>
	float Pitch;

	/// <summary>
	/// 偏航角(RY)
	/// </summary>
	float Yaw;

	/// <summary>
	/// 翻滚角(RZ)
	/// </summary>
	float Roll;

	/// <summary>
	/// 特征向量
	/// </summary>
	float Features[36];
};
