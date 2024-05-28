#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <primitives_map.h>
#include "pcl_common.h"
using namespace std;
using namespace pcl;

/// <summary>
/// 估算质心
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <returns>质心坐标点</returns>
Point3F* estimateCentroid(Point3F points[], const int length)
{
	//加载点云
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud, centroid);

	Point3F* point3F = new Point3F(centroid[0], centroid[1], centroid[2]);

	return point3F;
}

/// <summary>
/// 仿射变换
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="pose">位姿</param>
/// <returns>变换后点云</returns>
Point3Fs* affineTransform(Point3F points[], const int length, const Pose pose)
{
	const PointCloud<PointXYZ>::Ptr sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//创建仿射变换对象
	Eigen::Affine3f affine = Eigen::Affine3f::Identity();

	//平移
	affine.translation() << pose.X, pose.Y, pose.Z;

	//旋转
	const Eigen::AngleAxisf& rotationX = Eigen::AngleAxisf(pose.RX, Eigen::Vector3f::UnitX());
	const Eigen::AngleAxisf& rotationY = Eigen::AngleAxisf(pose.RY, Eigen::Vector3f::UnitY());
	const Eigen::AngleAxisf& rotationZ = Eigen::AngleAxisf(pose.RZ, Eigen::Vector3f::UnitZ());
	affine.rotate(rotationX);
	affine.rotate(rotationY);
	affine.rotate(rotationZ);

	//执行变换
	pcl::transformPointCloud(*sourceCloud, *targetCloud, affine);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}
