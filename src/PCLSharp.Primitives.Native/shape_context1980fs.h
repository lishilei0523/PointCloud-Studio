#pragma once
#include "shape_context1980f.h"

/// <summary>
/// 3DSC描述子集结构体
/// </summary>
struct ShapeContext1980Fs
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	ShapeContext1980Fs() = default;

	/// <summary>
	/// 创建3DSC描述子集结构体构造器
	/// </summary>
	/// <param name="descriptors">3DSC描述子集指针</param>
	/// <param name="length">长度</param>
	ShapeContext1980Fs(ShapeContext1980F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~ShapeContext1980Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// 3DSC描述子集指针
	/// </summary>
	ShapeContext1980F* Descriptors;

	/// <summary>
	/// 长度
	/// </summary>
	int Length;
};
