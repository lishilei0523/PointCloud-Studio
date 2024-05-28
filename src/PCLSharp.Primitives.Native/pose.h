#pragma once

/// <summary>
/// 位姿
/// </summary>
struct Pose
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Pose() = default;

	/// <summary>
	/// 创建位姿构造器
	/// </summary>
	/// <param name="x">X轴位置</param>
	/// <param name="y">Y轴位置</param>
	/// <param name="z">Z轴位置</param>
	/// <param name="rx">X轴旋转角度</param>
	/// <param name="ry">Y轴旋转角度</param>
	/// <param name="rz">Z轴旋转角度</param>
	Pose(const float& x, const float& y, const float& z, const float& rx, const float& ry, const float& rz)
		:X(x), Y(y), Z(z), RX(rx), RY(ry), RZ(rz)
	{

	}

	/// <summary>
	/// X轴位置
	/// </summary>
	float X;

	/// <summary>
	/// Y轴位置
	/// </summary>
	float Y;

	/// <summary>
	/// Z轴位置
	/// </summary>
	float Z;

	/// <summary>
	/// X轴旋转角度
	/// </summary>
	float RX;

	/// <summary>
	/// Y轴旋转角度
	/// </summary>
	float RY;

	/// <summary>
	/// Z轴旋转角度
	/// </summary>
	float RZ;
};
