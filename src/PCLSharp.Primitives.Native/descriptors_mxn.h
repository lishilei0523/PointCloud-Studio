#pragma once

/// <summary>
/// 特征描述子MxN矩阵
/// </summary>
struct DescriptorsMxN
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	DescriptorsMxN() = default;

	/// <summary>
	/// 创建特征描述子MxN矩阵构造器
	/// </summary>
	/// <param name="matrix">特征矩阵</param>
	/// <param name="rowsCount">行数</param>
	/// <param name="colsCount">列数</param>
	DescriptorsMxN(float** matrix, const int& rowsCount, const int& colsCount)
		:Matrix(matrix), RowsCount(rowsCount), ColsCount(colsCount)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~DescriptorsMxN()
	{
		//释放每行
		for (int rowIndex = 0; rowIndex < this->RowsCount; rowIndex++)
		{
			delete[] this->Matrix[rowIndex];
		}

		//释放整体
		delete[] this->Matrix;
	}

	/// <summary>
	/// 特征矩阵
	/// </summary>
	float** Matrix;

	/// <summary>
	/// 行数
	/// </summary>
	int RowsCount;

	/// <summary>
	/// 列数
	/// </summary>
	int ColsCount;
};
