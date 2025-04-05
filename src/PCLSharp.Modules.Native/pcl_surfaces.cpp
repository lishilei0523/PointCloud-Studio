#include <pcl/common/io.h>
#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <primitives_map.h>
#include "pcl_surfaces.h"
using namespace std;
using namespace pcl;

/// <summary>
/// 适用贪婪投影三角化
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="searchRadius">搜索半径</param>
/// <param name="mu">近邻点最远倍数</param>
/// <param name="maxNearestNeighbors">最大邻域数</param>
/// <param name="maxSurfaceAngle">偏离法向量最大角度</param>
/// <param name="minAngle">三角形最小角度</param>
/// <param name="maxAngle">三角形最大角度</param>
/// <param name="normalConsistency">保证法向量朝向一致</param>
/// <param name="threadsCount">线程数</param>
/// <returns>网格几何</returns>
MeshGeometry* applyGreedyProjection(Point3F points[], const int length, const int normalK, const float searchRadius, const float mu, const int maxNearestNeighbors, const double maxSurfaceAngle, const double minAngle, const double maxAngle, const bool normalConsistency, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<PointNormal>::Ptr cloudNormals = std::make_shared<PointCloud<PointNormal>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	const search::KdTree<PointNormal>::Ptr kdTreePN = std::make_shared<search::KdTree<PointNormal>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//连接点云与法向量
	concatenateFields(*cloud, *normals, *cloudNormals);

	//贪婪投影三角化
	PolygonMesh polygonMesh;
	GreedyProjectionTriangulation<PointNormal> greedyProjection;
	greedyProjection.setInputCloud(cloudNormals);
	greedyProjection.setSearchMethod(kdTreePN);
	greedyProjection.setSearchRadius(searchRadius);
	greedyProjection.setMu(mu);
	greedyProjection.setMaximumNearestNeighbors(maxNearestNeighbors);
	greedyProjection.setMaximumSurfaceAngle(deg2rad(maxSurfaceAngle));
	greedyProjection.setMinimumAngle(deg2rad(minAngle));
	greedyProjection.setMaximumAngle(deg2rad(maxAngle));
	greedyProjection.setNormalConsistency(normalConsistency);
	greedyProjection.reconstruct(polygonMesh);

	MeshGeometry* meshGeometry = pclsharp::toMeshGeometry(polygonMesh);

	return meshGeometry;
}

/// <summary>
/// 适用泊松重建
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="confidence">是否使用法向量置信</param>
/// <param name="degree">度数</param>
/// <param name="depth">树最大深度</param>
/// <param name="isoDivide">ISO等值面深度</param>
/// <param name="manifold">是否添加多边形重心</param>
/// <param name="outputPolygons">是否输出多边形网格</param>
/// <param name="samplesPerNode">八叉树样本点最小数量</param>
/// <param name="scale">重构立方体与样本边界立方体直径比率</param>
/// <param name="solverDivide">Gauss-Seidel迭代深度</param>
/// <param name="threadsCount">线程数</param>
/// <returns>网格几何</returns>
MeshGeometry* applyPoissonReconstruction(Point3F points[], const int length, const int normalK, const bool confidence, const int degree, const int depth, const int isoDivide, const bool manifold, const bool outputPolygons, const float samplesPerNode, const float scale, const int solverDivide, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<PointNormal>::Ptr cloudNormals = std::make_shared<PointCloud<PointNormal>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	const search::KdTree<PointNormal>::Ptr kdTreePN = std::make_shared<search::KdTree<PointNormal>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//连接点云与法向量
	concatenateFields(*cloud, *normals, *cloudNormals);

	//泊松重建
	PolygonMesh polygonMesh;
	Poisson<PointNormal> poisson;
	poisson.setInputCloud(cloudNormals);
	poisson.setSearchMethod(kdTreePN);
	poisson.setConfidence(confidence);
	poisson.setDegree(degree);
	poisson.setDepth(depth);
	poisson.setIsoDivide(isoDivide);
	poisson.setManifold(manifold);
	poisson.setOutputPolygons(outputPolygons);
	poisson.setSamplesPerNode(samplesPerNode);
	poisson.setScale(scale);
	poisson.setSolverDivide(solverDivide);
	poisson.performReconstruction(polygonMesh);

	MeshGeometry* meshGeometry = pclsharp::toMeshGeometry(polygonMesh);

	return meshGeometry;
}

/// <summary>
/// 适用移动立方体重建
/// </summary>
/// <param name="points">点集</param>
/// <param name="length">点集长度</param>
/// <param name="normalK">法向量K</param>
/// <param name="isoLevel">等值面值</param>
/// <param name="gridResolution">网格分辨率</param>
/// <param name="percentageExtendGrid">自由空间比例</param>
/// <param name="threadsCount">线程数</param>
/// <returns>网格几何</returns>
MeshGeometry* applyMarchingCubes(Point3F points[], const int length, const int normalK, const float isoLevel, const int gridResolution, const float percentageExtendGrid, const int threadsCount)
{
	const PointCloud<PointXYZ>::Ptr cloud = pclsharp::toPointCloud(points, length);
	const PointCloud<Normal>::Ptr normals = std::make_shared<PointCloud<Normal>>();
	const PointCloud<PointNormal>::Ptr cloudNormals = std::make_shared<PointCloud<PointNormal>>();
	const search::KdTree<PointXYZ>::Ptr kdTree = std::make_shared<search::KdTree<PointXYZ>>();
	const search::KdTree<PointNormal>::Ptr kdTreePN = std::make_shared<search::KdTree<PointNormal>>();

	//计算法向量
	NormalEstimationOMP<PointXYZ, Normal> normalEstimator;
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setSearchMethod(kdTree);
	normalEstimator.setKSearch(normalK);
	normalEstimator.setNumberOfThreads(threadsCount);
	normalEstimator.compute(*normals);

	//连接点云与法向量
	concatenateFields(*cloud, *normals, *cloudNormals);

	//移动立方体重建
	PolygonMesh polygonMesh;
	MarchingCubesHoppe<PointNormal> marchingCubes;
	marchingCubes.setInputCloud(cloudNormals);
	marchingCubes.setSearchMethod(kdTreePN);
	marchingCubes.setIsoLevel(isoLevel);
	marchingCubes.setGridResolution(gridResolution, gridResolution, gridResolution);
	marchingCubes.setPercentageExtendGrid(percentageExtendGrid);
	marchingCubes.reconstruct(polygonMesh);

	MeshGeometry* meshGeometry = pclsharp::toMeshGeometry(polygonMesh);

	return meshGeometry;
}
