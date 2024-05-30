#pragma once

/// <summary>
/// 配准结果
/// </summary>
struct AlignmentResult
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	AlignmentResult() = default;

	/// <summary>
	/// 创建配准结果构造器
	/// </summary>
	/// <param name="hasConverged">是否收敛</param>
	/// <param name="fitnessScore">拟合分数</param>
	AlignmentResult(const bool& hasConverged, const float& fitnessScore)
		:HasConverged(hasConverged), FitnessScore(fitnessScore), Matrix{ 0.0f }
	{

	}

	/// <summary>
	/// 是否收敛
	/// </summary>
	bool HasConverged;

	/// <summary>
	/// 拟合分数
	/// </summary>
	float FitnessScore;

	/// <summary>
	/// RT矩阵
	/// </summary>
	float Matrix[16];
};
