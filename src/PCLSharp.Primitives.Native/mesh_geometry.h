#pragma once
#include "point3f.h"

/// <summary>
/// 网格几何
/// </summary>
struct MeshGeometry
{
	/// <summary>
	/// 无参构造器
	/// </summary>
	MeshGeometry() = default;

	/// <summary>
	/// 创建网格几何构造器
	/// </summary>
	/// <param name="positions">位置集指针</param>
	/// <param name="positionsLength">位置集长度</param>
	/// <param name="triangleIndices">三角索引集指针</param>
	/// <param name="triangleIndicesLength">三角索引集长度</param>
	MeshGeometry(Point3F* positions, const int& positionsLength, int* triangleIndices, const int& triangleIndicesLength)
		:Positions(positions), PositionsLength(positionsLength), TriangleIndices(triangleIndices), TriangleIndicesLength(triangleIndicesLength)
	{

	}

	/// <summary>
	/// 析构函数
	/// </summary>
	~MeshGeometry()
	{
		delete[] this->Positions;
		delete[] this->TriangleIndices;
	}

	/// <summary>
	/// 位置集指针
	/// </summary>
	Point3F* Positions;

	/// <summary>
	/// 位置集长度
	/// </summary>
	int PositionsLength;

	/// <summary>
	/// 三角索引集指针
	/// </summary>
	int* TriangleIndices;

	/// <summary>
	/// 三角索引集长度
	/// </summary>
	int TriangleIndicesLength;
};
