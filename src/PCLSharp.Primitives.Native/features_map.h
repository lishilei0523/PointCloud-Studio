#pragma once
#ifdef _WIN32
#define EXPORT_CPP extern __declspec(dllexport)
#elif __linux__
#define EXPORT_CPP extern
#endif
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "narf36f.h"
#include "narf36fs.h"
#include "pfh_signature125f.h"
#include "pfh_signature125fs.h"
#include "fpfh_signature33f.h"
#include "fpfh_signature33fs.h"
#include "shape_context1980f.h"
#include "shape_context1980fs.h"
#include "shot352f.h"
#include "shot352fs.h"

namespace pclsharp
{
	/// <summary>
	/// NARF特征描述子映射NARF点云
	/// </summary>
	/// <param name="narf36Fs">NARF特征描述子集</param>
	/// <param name="length">长度</param>
	/// <returns>NARF点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::Narf36>::Ptr toPointCloud(Narf36F narf36Fs[], const int& length);

	/// <summary>
	/// NARF点云映射NARF特征描述子
	/// </summary>
	/// <param name="pointCloud">NARF点云</param>
	/// <returns>NARF特征描述子集</returns>
	EXPORT_CPP Narf36Fs* toNarf36Fs(const pcl::PointCloud<pcl::Narf36>& pointCloud);

	/// <summary>
	/// PFH特征描述子映射PFH点云
	/// </summary>
	/// <param name="signature125Fs">PFH特征描述子集</param>
	/// <param name="length">长度</param>
	/// <returns>PFH点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::PFHSignature125>::Ptr toPointCloud(PFHSignature125F signature125Fs[], const int& length);

	/// <summary>
	/// PFH点云映射PFH特征描述子
	/// </summary>
	/// <param name="pointCloud">PFH点云</param>
	/// <returns>PFH特征描述子集</returns>
	EXPORT_CPP PFHSignature125Fs* toPFHSignature125Fs(const pcl::PointCloud<pcl::PFHSignature125>& pointCloud);

	/// <summary>
	/// FPFH特征描述子映射FPFH点云
	/// </summary>
	/// <param name="signature33Fs">FPFH特征描述子集</param>
	/// <param name="length">长度</param>
	/// <returns>FPFH点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::FPFHSignature33>::Ptr toPointCloud(FPFHSignature33F signature33Fs[], const int& length);

	/// <summary>
	/// FPFH点云映射FPFH特征描述子
	/// </summary>
	/// <param name="pointCloud">FPFH点云</param>
	/// <returns>FPFH特征描述子集</returns>
	EXPORT_CPP FPFHSignature33Fs* toFPFHSignature33Fs(const pcl::PointCloud<pcl::FPFHSignature33>& pointCloud);

	/// <summary>
	/// 3DSC特征描述子映射3DSC点云
	/// </summary>
	/// <param name="shapeContext1980Fs">3DSC特征描述子集</param>
	/// <param name="length">长度</param>
	/// <returns>3DSC点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::ShapeContext1980>::Ptr toPointCloud(ShapeContext1980F shapeContext1980Fs[], const int& length);

	/// <summary>
	/// 3DSC点云映射3DSC特征描述子
	/// </summary>
	/// <param name="pointCloud">3DSC点云</param>
	/// <returns>3DSC特征描述子集</returns>
	EXPORT_CPP ShapeContext1980Fs* toShapeContext1980Fs(const pcl::PointCloud<pcl::ShapeContext1980>& pointCloud);

	/// <summary>
	/// SHOT特征描述子映射SHOT点云
	/// </summary>
	/// <param name="shot352Fs">SHOT特征描述子集</param>
	/// <param name="length">长度</param>
	/// <returns>SHOT点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::SHOT352>::Ptr toPointCloud(Shot352F shot352Fs[], const int& length);

	/// <summary>
	/// SHOT点云映射SHOT特征描述子
	/// </summary>
	/// <param name="pointCloud">SHOT点云</param>
	/// <returns>SHOT特征描述子集</returns>
	EXPORT_CPP Shot352Fs* toShot352Fs(const pcl::PointCloud<pcl::SHOT352>& pointCloud);
}
