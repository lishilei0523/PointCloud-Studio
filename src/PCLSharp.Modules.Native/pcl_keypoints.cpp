#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3D.h>
#include <pcl/keypoints/susan.h>
#include <pcl/features/range_image_border_extractor.h>
#include <primitives_map.h>
#include "pcl_keypoints.h"
using namespace std;
using namespace pcl;

/* SIFT关键点计算必须加这段 */
namespace pcl
{
	template<>
	struct SIFTKeypointFieldSelector<PointXYZ>
	{
		inline float operator()(const PointXYZ& point) const
		{
			return point.z;
		}
	};
}

/// <summary>
/// 检测NARF关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="angularResolution">角度分辨率</param>
/// <param name="maxAngleWidth">传感器水平边界角度</param>
/// <param name="maxAngleHeight">传感器垂直边界角度</param>
/// <param name="noiseLevel">所有与最近点的最大距离</param>
/// <param name="minRange">最小可见范围</param>
/// <param name="borderSize">边界尺寸</param>
/// <param name="supportSize">计算范围半径</param>
/// <returns>NARF关键点集</returns>
Point3Fs* detectNARF(Point3F points[], const int length, const float angularResolution, const float maxAngleWidth, const float maxAngleHeight, const float noiseLevel, const float minRange, const int borderSize, const float supportSize)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();
	const RangeImage::Ptr rangeImage = std::make_shared<RangeImage>();

	//生成深度图
	const Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Affine3f::Identity());
	rangeImage->createFromPointCloud(*cloud, deg2rad(angularResolution), deg2rad(maxAngleWidth), deg2rad(maxAngleHeight), sensorPose, RangeImage::CAMERA_FRAME, noiseLevel, minRange, borderSize);

	//提取NARF关键点索引
	PointCloud<int> keyPointsIndices;
	RangeImageBorderExtractor rangeImageBorderExtractor;
	NarfKeypoint narfDetector = NarfKeypoint(&rangeImageBorderExtractor);
	narfDetector.setRangeImage(&*rangeImage);
	NarfKeypoint::Parameters& detectorParameters = narfDetector.getParameters();
	detectorParameters.support_size = supportSize;
	narfDetector.compute(keyPointsIndices);

	//提取NARF关键点
	const size_t& keyPointsCount = keyPointsIndices.points.size();
	keyPoints->points.resize(keyPointsCount);
	for (int index = 0; index < keyPointsCount; index++)
	{
		keyPoints->points[index].getVector3fMap() = rangeImage->points[keyPointsIndices.points[index]].getVector3fMap();
	}

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}

/// <summary>
/// 检测ISS关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="salientRadius">显著半径</param>
/// <param name="nonMaxRadius">非极大值抑制半径</param>
/// <param name="threshold21">二一特征值比上限</param>
/// <param name="threshold32">三二特征值比上限</param>
/// <param name="minNeighborsCount">最小邻域点数</param>
/// <param name="threadsCount">线程数</param>
/// <returns>ISS关键点集</returns>
Point3Fs* detectISS(Point3F points[], const int length, const float salientRadius, const float nonMaxRadius, const float threshold21, const float threshold32, const int minNeighborsCount, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr& cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();

	//提取ISS关键点
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	ISSKeypoint3D<PointXYZ, PointXYZ> issDetector;
	issDetector.setInputCloud(cloud);
	issDetector.setSearchMethod(kdTree);
	issDetector.setSalientRadius(salientRadius);
	issDetector.setNonMaxRadius(nonMaxRadius);
	issDetector.setThreshold21(threshold21);
	issDetector.setThreshold32(threshold32);
	issDetector.setMinNeighbors(minNeighborsCount);
	issDetector.setNumberOfThreads(threadsCount);
	issDetector.compute(*keyPoints);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}

/// <summary>
/// 检测SIFT关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="minScale">尺度空间最小标准偏差</param>
/// <param name="octavesCount">金字塔组数量</param>
/// <param name="scalesPerOctaveCount">每组金字塔计算尺度</param>
/// <param name="minContrast">限制关键点检测阈值</param>
/// <returns>SIFT关键点集</returns>
Point3Fs* detectSIFT(Point3F points[], const int length, const float minScale, const int octavesCount, const int scalesPerOctaveCount, const float minContrast)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();

	//提取SIFT关键点
	search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	PointCloud<PointWithScale> result;
	SIFTKeypoint<PointXYZ, PointWithScale> siftDetector;
	siftDetector.setInputCloud(cloud);
	siftDetector.setSearchMethod(kdTree);
	siftDetector.setScales(minScale, octavesCount, scalesPerOctaveCount);
	siftDetector.setMinimumContrast(minContrast);
	siftDetector.compute(result);
	pcl::copyPointCloud(result, *keyPoints);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}

/// <summary>
/// 检测Harris关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="nonMaxSupression">非极大值抑制</param>
/// <param name="radius">搜索半径</param>
/// <param name="threshold">感兴趣阈值</param>
/// <returns>Harris关键点集</returns>
Point3Fs* detectHarris(Point3F points[], const int length, const bool nonMaxSupression, const float radius, const float threshold)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();

	//提取Harris关键点
	PointCloud<PointXYZI> result;
	HarrisKeypoint3D<PointXYZ, PointXYZI> harrisDetector;
	harrisDetector.setInputCloud(cloud);
	harrisDetector.setNonMaxSupression(nonMaxSupression);
	harrisDetector.setRadius(radius);
	harrisDetector.setThreshold(threshold);
	harrisDetector.compute(result);
	pcl::copyPointCloud(result, *keyPoints);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}

/// <summary>
/// 检测SUSAN关键点
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="nonMaxSupression">非极大值抑制</param>
/// <param name="radius">搜索半径</param>
/// <param name="distanceThreshold">距离阈值</param>
/// <param name="angularThreshold">角度阈值</param>
/// <param name="intensityThreshold">强度阈值</param>
/// <returns>SUSAN关键点集</returns>
Point3Fs* detectSUSAN(Point3F points[], const int length, const bool nonMaxSupression, const float radius, const float distanceThreshold, const float angularThreshold, const float intensityThreshold)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<PointXYZ>::Ptr keyPoints = std::make_shared<PointCloud<PointXYZ>>();

	//提取SUSAN关键点
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	SUSANKeypoint<PointXYZ, PointXYZ> susanDetector;
	susanDetector.setInputCloud(cloud);
	susanDetector.setSearchMethod(kdTree);
	susanDetector.setNonMaxSupression(nonMaxSupression);
	susanDetector.setRadius(radius);
	susanDetector.setDistanceThreshold(distanceThreshold);
	susanDetector.setAngularThreshold(angularThreshold);
	susanDetector.setIntensityThreshold(intensityThreshold);
	susanDetector.compute(*keyPoints);

	Point3Fs* point3Fs = pclsharp::toPoint3Fs(*keyPoints);

	return point3Fs;
}
