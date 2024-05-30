#include "pcl_dispose.h"

/// <summary>
/// 释放坐标点
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3F(const Point3F* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点集
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Fs(const Point3Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点集分组
/// </summary>
/// <param name="pointer">二级指针</param>
/// <param name="groupCount">分组数</param>
void disposePoint3FsGroup(const Point3Fs** pointer, const int groupCount)
{
	for (int groupIndex = 0; groupIndex < groupCount; groupIndex++)
	{
		delete pointer[groupIndex];
	}
	delete pointer;
}

/// <summary>
/// 释放法向量
/// </summary>
/// <param name="pointer">指针</param>
void disposeNormal3F(const Normal3F* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放法向量集
/// </summary>
/// <param name="pointer">指针</param>
void disposeNormal3Fs(const Normal3Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放法向量集分组
/// </summary>
/// <param name="pointer">二级指针</param>
/// <param name="groupCount">分组数</param>
void disposeNormal3FsGroup(const Point3Fs** pointer, const int groupCount)
{
	for (int groupIndex = 0; groupIndex < groupCount; groupIndex++)
	{
		delete pointer[groupIndex];
	}
	delete pointer;
}

/// <summary>
/// 释放坐标点法向量
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Normal3(const Point3Normal3* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点法向量集
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Normal3s(const Point3Normal3s* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点法向量集分组
/// </summary>
/// <param name="pointer">二级指针</param>
/// <param name="groupCount">分组数</param>
void disposePoint3Normal3sGroup(const Point3Fs** pointer, const int groupCount)
{
	for (int groupIndex = 0; groupIndex < groupCount; groupIndex++)
	{
		delete pointer[groupIndex];
	}
	delete pointer;
}

/// <summary>
/// 释放坐标点颜色
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Color4(const Point3Color4* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点颜色集
/// </summary>
/// <param name="pointer">指针</param>
void disposePoint3Color4s(const Point3Color4s* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放坐标点颜色集分组
/// </summary>
/// <param name="pointer">二级指针</param>
/// <param name="groupCount">分组数</param>
void disposePoint3Color4sGroup(const Point3Fs** pointer, const int groupCount)
{
	for (int groupIndex = 0; groupIndex < groupCount; groupIndex++)
	{
		delete pointer[groupIndex];
	}
	delete pointer;
}

/// <summary>
/// 释放NARF描述子
/// </summary>
/// <param name="pointer">指针</param>
void disposeNarf36F(const Narf36F* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放NARF描述子集
/// </summary>
/// <param name="pointer">指针</param>
void disposeNarf36Fs(const Narf36Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放PFH描述子
/// </summary>
/// <param name="pointer">指针</param>
void disposePFHSignature125F(const PFHSignature125F* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放PFH描述子集
/// </summary>
/// <param name="pointer">指针</param>
void disposePFHSignature125Fs(const PFHSignature125Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放FPFH描述子
/// </summary>
/// <param name="pointer">指针</param>
void disposeFPFHSignature33F(const FPFHSignature33F* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放FPFH描述子集
/// </summary>
/// <param name="pointer">指针</param>
void disposeFPFHSignature33Fs(const FPFHSignature33Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放3DSC描述子
/// </summary>
/// <param name="pointer">指针</param>
void disposeShapeContext1980F(const ShapeContext1980F* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放3DSC描述子集
/// </summary>
/// <param name="pointer">指针</param>
void disposeShapeContext1980Fs(const ShapeContext1980Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放SHOT描述子
/// </summary>
/// <param name="pointer">指针</param>
void disposeShot352F(const Shot352F* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放SHOT描述子集
/// </summary>
/// <param name="pointer">指针</param>
void disposeShot352Fs(const Shot352Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// 释放配准结果
/// </summary>
/// <param name="pointer">指针</param>
void disposeAlignmentResult(const AlignmentResult* pointer)
{
	delete pointer;
}
