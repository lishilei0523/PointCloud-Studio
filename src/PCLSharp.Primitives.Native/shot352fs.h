#pragma once
#include "shot352f.h"

/// <summary>
/// SHOT描述子集
/// </summary>
struct Shot352Fs
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Shot352Fs() = default;

	/// <summary>
	/// 创建SHOT描述子集构造器
	/// </summary>
	/// <param name="descriptors">SHOT描述子集指针</param>
	/// <param name="length">长度</param>
	Shot352Fs(Shot352F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~Shot352Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// SHOT描述子集指针
	/// </summary>
	Shot352F* Descriptors;

	/// <summary>
	/// 长度
	/// </summary>
	int Length;
};
