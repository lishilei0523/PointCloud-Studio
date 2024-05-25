#include "features_map.h"
using namespace std;
using namespace pcl;

/// <summary>
/// NARF特征描述子集映射NARF点云
/// </summary>
/// <param name="narf36Fs">NARF特征描述子集</param>
/// <param name="length">长度</param>
/// <returns>NARF点云</returns>
PointCloud<Narf36>::Ptr toPointCloud(Narf36F narf36Fs[], const int& length)
{
	const size_t& colsCount = 36;
	const PointCloud<Narf36>::Ptr& pointCloud = std::make_shared<PointCloud<Narf36>>();
	for (int rowIndex = 0; rowIndex < length; rowIndex++)
	{
		const Narf36F& narf36F = narf36Fs[rowIndex];
		Narf36 narf36 = Narf36(narf36F.X, narf36F.Y, narf36F.Z, narf36F.Roll, narf36F.Pitch, narf36F.Yaw);
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			narf36.descriptor[colIndex] = narf36F.Features[colIndex];
		}
		pointCloud->push_back(narf36);
	}

	return pointCloud;
}

/// <summary>
/// NARF点云映射NARF特征描述子
/// </summary>
/// <param name="pointCloud">点云</param>
/// <returns>NARF特征描述子</returns>
Narf36Fs* pclsharp::toNarf36Fs(const PointCloud<Narf36>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	const size_t& colsCount = 36;
	Narf36F* descriptors = new Narf36F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const Narf36& narf36 = pointCloud.points[rowIndex];
		Narf36F narf36F = Narf36F(narf36.x, narf36.y, narf36.z, narf36.pitch, narf36.yaw, narf36.roll);
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const float& value = narf36.descriptor[colIndex];
			narf36F.Features[colIndex] = value;
		}
		descriptors[rowIndex] = narf36F;
	}

	Narf36Fs* narf36Fs = new Narf36Fs(descriptors, static_cast<int>(rowsCount));

	return narf36Fs;
}

/// <summary>
/// PFH点云映射PFH特征描述子
/// </summary>
/// <param name="pointCloud">点云</param>
/// <returns>PFH特征描述子</returns>
PFHSignature125Fs* pclsharp::toPFHSignature125Fs(const PointCloud<PFHSignature125>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	const size_t& colsCount = 125;
	PFHSignature125F* descriptors = new PFHSignature125F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const PFHSignature125& signature125 = pointCloud.points[rowIndex];
		PFHSignature125F signature125F = PFHSignature125F();
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const float& value = signature125.histogram[colIndex];
			signature125F.Features[colIndex] = value;
		}
		descriptors[rowIndex] = signature125F;
	}

	PFHSignature125Fs* signature125Fs = new PFHSignature125Fs(descriptors, static_cast<int>(rowsCount));

	return signature125Fs;
}

/// <summary>
/// FPFH点云映射FPFH特征描述子
/// </summary>
/// <param name="pointCloud">点云</param>
/// <returns>FPFH特征描述子</returns>
FPFHSignature33Fs* pclsharp::toFPFHSignature33Fs(const PointCloud<FPFHSignature33>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	const size_t& colsCount = 33;
	FPFHSignature33F* descriptors = new FPFHSignature33F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const FPFHSignature33& signature33 = pointCloud.points[rowIndex];
		FPFHSignature33F signature33F = FPFHSignature33F();
		for (int colIndex = 0; colIndex < colsCount; colIndex++)
		{
			const float& value = signature33.histogram[colIndex];
			signature33F.Features[colIndex] = value;
		}
		descriptors[rowIndex] = signature33F;
	}

	FPFHSignature33Fs* signature33Fs = new FPFHSignature33Fs(descriptors, static_cast<int>(rowsCount));

	return signature33Fs;
}

/// <summary>
/// 3DSC点云映射3DSC特征描述子
/// </summary>
/// <param name="pointCloud">点云</param>
/// <returns>3DSC特征描述子</returns>
ShapeContext1980Fs* pclsharp::toShapeContext1980Fs(const PointCloud<ShapeContext1980>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	ShapeContext1980F* descriptors = new ShapeContext1980F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const ShapeContext1980& shapeContext1980 = pointCloud.points[rowIndex];
		ShapeContext1980F shapeContext1980F = ShapeContext1980F();
		for (int i = 0; i < 9; i++)
		{
			shapeContext1980F.RF[i] = shapeContext1980.rf[i];
		}
		for (int i = 0; i < 1980; i++)
		{
			shapeContext1980F.Features[i] = shapeContext1980.descriptor[i];
		}
		descriptors[rowIndex] = shapeContext1980F;
	}

	ShapeContext1980Fs* shapeContext1980Fs = new ShapeContext1980Fs(descriptors, static_cast<int>(rowsCount));

	return shapeContext1980Fs;
}

/// <summary>
/// SHOT点云映射SHOT特征描述子
/// </summary>
/// <param name="pointCloud">点云</param>
/// <returns>SHOT特征描述子</returns>
Shot352Fs* pclsharp::toShot352Fs(const PointCloud<SHOT352>& pointCloud)
{
	const size_t& rowsCount = pointCloud.size();
	Shot352F* descriptors = new Shot352F[rowsCount];
	for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
	{
		const SHOT352& shot352 = pointCloud.points[rowIndex];
		Shot352F shot352F = Shot352F();
		for (int i = 0; i < 9; i++)
		{
			shot352F.RF[i] = shot352.rf[i];
		}
		for (int i = 0; i < 352; i++)
		{
			shot352F.Features[i] = shot352.descriptor[i];
		}
		descriptors[rowIndex] = shot352F;
	}

	Shot352Fs* shot352Fs = new Shot352Fs(descriptors, static_cast<int>(rowsCount));

	return shot352Fs;
}
