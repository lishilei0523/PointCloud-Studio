#pragma once
#include "narf36f.h"

/// <summary>
/// NARF描述子集
/// </summary>
struct Narf36Fs
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Narf36Fs() = default;

	/// <summary>
	/// 创建NARF描述子集构造器
	/// </summary>
	/// <param name="descriptors">NARF描述子集指针</param>
	/// <param name="length">长度</param>
	Narf36Fs(Narf36F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~Narf36Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// NARF描述子集指针
	/// </summary>
	Narf36F* Descriptors;

	/// <summary>
	/// 长度
	/// </summary>
	int Length;
};
