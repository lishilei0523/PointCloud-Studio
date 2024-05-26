#pragma once
#include "normal3f.h"

/// <summary>
/// 法向量集
/// </summary>
struct Normal3Fs
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	Normal3Fs() = default;

	/// <summary>
	/// 创建法向量集构造器
	/// </summary>
	/// <param name="normals">法向量集指针</param>
	/// <param name="length">长度</param>
	Normal3Fs(Normal3F* normals, const int& length)
		:Normals(normals), Length(length)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~Normal3Fs()
	{
		delete[] this->Normals;
	}

	/// <summary>
	/// 法向量集指针
	/// </summary>
	Normal3F* Normals;

	/// <summary>
	/// 长度
	/// </summary>
	int Length;
};
