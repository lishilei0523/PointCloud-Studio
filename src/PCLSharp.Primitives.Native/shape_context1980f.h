#pragma once

/// <summary>
/// 3DSC描述子
/// </summary>
struct ShapeContext1980F
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	ShapeContext1980F()
		:RF{ 0.0f }, Features{ 0.0f }
	{

	}

	/// <summary>
	/// RF
	/// </summary>
	float RF[9];

	/// <summary>
	/// 特征向量
	/// </summary>
	float Features[1980];
};
