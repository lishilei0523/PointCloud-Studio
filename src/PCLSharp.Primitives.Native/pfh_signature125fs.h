#pragma once
#include "pfh_signature125f.h"

/// <summary>
/// PFH描述子集结构体
/// </summary>
struct PFHSignature125Fs
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	PFHSignature125Fs() = default;

	/// <summary>
	/// 创建PFH描述子集结构体构造器
	/// </summary>
	/// <param name="descriptors">PFH描述子集指针</param>
	/// <param name="length">长度</param>
	PFHSignature125Fs(PFHSignature125F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~PFHSignature125Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// PFH描述子集指针
	/// </summary>
	PFHSignature125F* Descriptors;

	/// <summary>
	/// 长度
	/// </summary>
	int Length;
};
