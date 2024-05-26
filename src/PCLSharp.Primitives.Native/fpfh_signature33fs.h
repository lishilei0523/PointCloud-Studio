#pragma once
#include "fpfh_signature33f.h"

/// <summary>
/// FPFH描述子集
/// </summary>
struct FPFHSignature33Fs
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	FPFHSignature33Fs() = default;

	/// <summary>
	/// 创建FPFH描述子集构造器
	/// </summary>
	/// <param name="descriptors">FPFH描述子集指针</param>
	/// <param name="length">长度</param>
	FPFHSignature33Fs(FPFHSignature33F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~FPFHSignature33Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// FPFH描述子集指针
	/// </summary>
	FPFHSignature33F* Descriptors;

	/// <summary>
	/// 长度
	/// </summary>
	int Length;
};
