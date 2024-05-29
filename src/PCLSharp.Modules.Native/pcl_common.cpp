#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/boundary.h>
#include <pcl/surface/convex_hull.h>
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

/// <summary>
/// 长方体剪裁
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="minPoint">最小坐标点</param>
/// <param name="maxPoint">最大坐标点</param>
/// <param name="negative">true: 剪裁/false: 保留</param>
/// <returns>剪裁后点云</returns>
Point3Fs* cropBox(Point3F points[], const int length, const Point3F minPoint, const Point3F maxPoint, const bool negative)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const Eigen::Vector4f& min = Eigen::Vector4f(minPoint.X, minPoint.Y, minPoint.Z, 1.0f);
	const Eigen::Vector4f& max = Eigen::Vector4f(maxPoint.X, maxPoint.Y, maxPoint.Z, 1.0f);

	//长方体剪裁
	CropBox<PointXYZ> cropBox;
	cropBox.setInputCloud(sourceCloud);
	cropBox.setMin(min);
	cropBox.setMax(max);
	cropBox.setNegative(negative);
	cropBox.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 凸包剪裁
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="contourPoints">轮廓点集</param>
/// <param name="contourLength">轮廓点集长度</param>
/// <param name="dimensionsCount">维度数</param>
/// <returns>剪裁后点云</returns>
Point3Fs* cropConvexHull(Point3F points[], int length, Point3F contourPoints[], int contourLength, int dimensionsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//定义顶点
	PointCloud<PointXYZ>::Ptr vertices = std::make_shared<PointCloud<PointXYZ>>();
	for (int i = 0; i < contourLength; i++)
	{
		const Point3F& point3F = contourPoints[i];
		vertices->push_back(PointXYZ(point3F.X, point3F.Y, point3F.Z));
	}

	//构造凸包
	PointCloud<PointXYZ>::Ptr hullCloud = std::make_shared<PointCloud<PointXYZ>>();
	vector<Vertices> hullIndices;
	ConvexHull<PointXYZ> convexHull;
	convexHull.setInputCloud(vertices);
	convexHull.setDimension(dimensionsCount);
	convexHull.reconstruct(*hullCloud, hullIndices);

	//凸包裁剪
	CropHull<PointXYZ> cropHull;
	cropHull.setInputCloud(sourceCloud);
	cropHull.setHullCloud(hullCloud);
	cropHull.setHullIndices(hullIndices);
	cropHull.setDim(dimensionsCount);
	cropHull.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 投射平面
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="a">平面方程系数a</param>
/// <param name="b">平面方程系数b</param>
/// <param name="c">平面方程系数c</param>
/// <param name="d">平面方程系数d</param>
/// <returns>投射后点云</returns>
/// <remarks>平面方程: ax + by +cz + d = 0</remarks>
Point3Fs* projectPlane(Point3F points[], const int length, const float a, const float b, const float c, const float d)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();

	//定义平面ax + by +cz + d = 0
	const ModelCoefficients::Ptr coefficients = std::make_shared<ModelCoefficients>();
	coefficients->values.resize(4);
	coefficients->values[0] = a;
	coefficients->values[1] = b;
	coefficients->values[2] = c;
	coefficients->values[3] = d;

	//投射平面
	ProjectInliers<PointXYZ> projectInliers;
	projectInliers.setInputCloud(sourceCloud);
	projectInliers.setModelType(pcl::SACMODEL_PLANE);
	projectInliers.setModelCoefficients(coefficients);
	projectInliers.filter(*targetCloud);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 提取边框
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <returns>边框点云</returns>
Point3Fs* extractBorder(Point3F points[], const int length)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const RangeImage::Ptr rangeImage = std::make_shared<RangeImage>();
	const PointCloud<BorderDescription>::Ptr borderDescriptions = std::make_shared<PointCloud<BorderDescription>>();

	//设置参数
	constexpr float angularResolution = 0.5f;
	constexpr float maxAngleWidth = 360.0f;
	constexpr float maxAngleHeight = 180.0f;
	const Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Affine3f::Identity());
	constexpr RangeImage::CoordinateFrame coordinateFrame = RangeImage::CAMERA_FRAME;
	constexpr float noiseLevel = 0.0f;
	constexpr float minRange = 0.0f;
	constexpr int borderSize = 0.0f;

	//创建深度图
	rangeImage->createFromPointCloud(*sourceCloud, deg2rad(angularResolution), deg2rad(maxAngleWidth), deg2rad(maxAngleHeight), sensorPose, coordinateFrame, noiseLevel, minRange, borderSize);
	rangeImage->setUnseenToMaxRange();

	//计算边框
	RangeImageBorderExtractor borderExtractor(&*rangeImage);
	borderExtractor.compute(*borderDescriptions);

	//提取边框
	const PointCloud<PointWithRange>::Ptr borderPoints = std::make_shared<PointCloud<PointWithRange>>();
	for (int i = 0; i < rangeImage->height; i++)
	{
		for (int j = 0; j < rangeImage->width; j++)
		{
			if (borderDescriptions->points[i * rangeImage->width + j].traits[BORDER_TRAIT__OBSTACLE_BORDER])
			{
				borderPoints->points.push_back(rangeImage->points[i * rangeImage->width + j]);
			}
		}
	}

	pcl::copyPointCloud(*borderPoints, *targetCloud);
	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}

/// <summary>
/// 提取边界
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="featureRadius">特征半径</param>
/// <param name="angleThreshold">角度阈值</param>
/// <param name="threadsCount">线程数</param>
/// <returns>边界点云</returns>
Point3Fs* extractBoundary(Point3F points[], const  int length, const  int normalK, const  float featureRadius, const  float angleThreshold, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& sourceCloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr targetCloud = std::make_shared<PointCloud<PointXYZ>>();
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<Boundary>::Ptr boundaries = std::make_shared<PointCloud<Boundary>>();
	search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(sourceCloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//提取边界
	BoundaryEstimation<PointXYZ, Normal, Boundary> boundaryComputer;
	boundaryComputer.setInputCloud(sourceCloud);
	boundaryComputer.setInputNormals(normals);
	boundaryComputer.setSearchMethod(kdTree);
	boundaryComputer.setRadiusSearch(featureRadius);
	boundaryComputer.setAngleThreshold(angleThreshold);
	boundaryComputer.compute(*boundaries);

	//边界转换点云
	for (int i = 0; i < sourceCloud->points.size(); i++)
	{
		if (boundaries->points[i].boundary_point > 0)
		{
			targetCloud->push_back(sourceCloud->points[i]);
		}
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*targetCloud);

	return point3Fs;
}
