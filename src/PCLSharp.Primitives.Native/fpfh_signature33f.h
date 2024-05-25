#pragma once

/// <summary>
/// FPFH描述子
/// </summary>
struct FPFHSignature33F
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	FPFHSignature33F()
		:Features{ 0.0f }
	{

	}

	/// <summary>
	/// 特征向量
	/// </summary>
	float Features[33];
};
