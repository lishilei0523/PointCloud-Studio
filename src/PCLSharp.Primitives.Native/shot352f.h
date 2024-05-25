#pragma once

/// <summary>
/// SHOT描述子
/// </summary>
struct Shot352F
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Shot352F()
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
	float Features[352];
};
