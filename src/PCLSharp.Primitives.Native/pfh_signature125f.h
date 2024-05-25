#pragma once

/// <summary>
/// PFH描述子
/// </summary>
struct PFHSignature125F
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	PFHSignature125F()
		:Features{ 0.0f }
	{

	}

	/// <summary>
	/// 特征向量
	/// </summary>
	float Features[125];
};
