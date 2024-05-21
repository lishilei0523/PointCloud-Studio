#include "primitives_map.h"
using namespace std;
using namespace pcl;

/// <summary>
/// 坐标点映射PointXYZ
/// </summary>
/// <param name="point3F">3D坐标点</param>
/// <returns>PointXYZ</returns>
PointXYZ pclsharp::toPointXYZ(const Point3F& point3F)
{
	const PointXYZ& pointXYZ = PointXYZ(point3F.X, point3F.Y, point3F.Z);

	return pointXYZ;
}

/// <summary>
/// 坐标点集映射点云
/// </summary>
/// <param name="point3Fs">坐标点集</param>
/// <param name="length">长度</param>
/// <returns>点云</returns>
PointCloud<PointXYZ> pclsharp::toPointCloud(Point3F point3Fs[], const int& length)
{
	PointCloud<PointXYZ> pointXYZs;
	for (int i = 0; i < length; i++)
	{
		const Point3F& point3F = point3Fs[i];
		PointXYZ pointXYZ = PointXYZ(point3F.X, point3F.Y, point3F.Z);
		pointXYZs.push_back(pointXYZ);
	}

	return pointXYZs;
}

/// <summary>
/// 点云映射坐标点集结构体
/// </summary>
/// <param name="pointCloud">点云</param>
/// <returns>坐标点集结构体</returns>
Point3Fs* pclsharp::toPoint3Fs(const PointCloud<PointXYZ>& pointCloud)
{
	const size_t length = pointCloud.size();
	Point3F* points = new Point3F[length];
	for (int i = 0; i < length; i++)
	{
		const PointXYZ& pointXYZ = pointCloud.points[i];
		points[i] = Point3F(pointXYZ.x, pointXYZ.y, pointXYZ.z);
	}

	Point3Fs* point3Fs = new Point3Fs(points, static_cast<int>(length));

	return point3Fs;
}
