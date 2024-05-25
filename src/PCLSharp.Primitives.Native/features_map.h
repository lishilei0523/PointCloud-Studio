#pragma once
#define EXPORT_CPP extern __declspec(dllexport)
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
	/// NARF特征描述子集映射NARF点云
	/// </summary>
	/// <param name="narf36Fs">NARF特征描述子集</param>
	/// <param name="length">长度</param>
	/// <returns>NARF点云</returns>
	EXPORT_CPP pcl::PointCloud<pcl::Narf36>::Ptr toPointCloud(Narf36F narf36Fs[], const int& length);

	/// <summary>
	/// NARF点云映射NARF特征描述子
	/// </summary>
	/// <param name="pointCloud">点云</param>
	/// <returns>NARF特征描述子</returns>
	EXPORT_CPP Narf36Fs* toNarf36Fs(const pcl::PointCloud<pcl::Narf36>& pointCloud);

	/// <summary>
	/// PFH点云映射PFH特征描述子
	/// </summary>
	/// <param name="pointCloud">点云</param>
	/// <returns>PFH特征描述子</returns>
	EXPORT_CPP PFHSignature125Fs* toPFHSignature125Fs(const pcl::PointCloud<pcl::PFHSignature125>& pointCloud);

	/// <summary>
	/// FPFH点云映射FPFH特征描述子
	/// </summary>
	/// <param name="pointCloud">点云</param>
	/// <returns>FPFH特征描述子</returns>
	EXPORT_CPP FPFHSignature33Fs* toFPFHSignature33Fs(const pcl::PointCloud<pcl::FPFHSignature33>& pointCloud);

	/// <summary>
	/// 3DSC点云映射3DSC特征描述子
	/// </summary>
	/// <param name="pointCloud">点云</param>
	/// <returns>3DSC特征描述子</returns>
	EXPORT_CPP ShapeContext1980Fs* toShapeContext1980Fs(const pcl::PointCloud<pcl::ShapeContext1980>& pointCloud);

	/// <summary>
	/// SHOT点云映射SHOT特征描述子
	/// </summary>
	/// <param name="pointCloud">点云</param>
	/// <returns>SHOT特征描述子</returns>
	EXPORT_CPP Shot352Fs* toShot352Fs(const pcl::PointCloud<pcl::SHOT352>& pointCloud);
}
