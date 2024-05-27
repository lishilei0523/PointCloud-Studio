using PCLSharp.Primitives.Features;
using PCLSharp.Primitives.Models;
using System.Collections.Generic;

namespace PCLSharp.Modules.Interfaces
{
    /// <summary>
    /// 点云特征接口
    /// </summary>
    public interface ICloudFeatures
    {
        #region # 计算NARF特征 —— Narf36F[] ComputeNARF(IEnumerable<Point3F> points, float angularResolution...
        /// <summary>
        /// 计算NARF特征
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="angularResolution">角度分辨率</param>
        /// <param name="maxAngleWidth">传感器水平边界角度</param>
        /// <param name="maxAngleHeight">传感器垂直边界角度</param>
        /// <param name="noiseLevel">所有与最近点的最大距离</param>
        /// <param name="minRange">最小可见范围</param>
        /// <param name="borderSize">边界尺寸</param>
        /// <param name="supportSize">计算范围半径</param>
        /// <param name="rotationInvariant">旋转不变性</param>
        /// <returns>NARF特征描述子集</returns>
        Narf36F[] ComputeNARF(IEnumerable<Point3F> points, float angularResolution, float maxAngleWidth, float maxAngleHeight, float noiseLevel, float minRange, int borderSize, float supportSize, bool rotationInvariant);
        #endregion

        #region # 计算PFH特征 —— PFHSignature125F[] ComputePFH(IEnumerable<Point3F> points, int normalK...
        /// <summary>
        /// 计算PFH特征
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="featureK">特征K</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>PFH特征描述子集</returns>
        /// <remarks>特征K值要大于法向量K值</remarks>
        PFHSignature125F[] ComputePFH(IEnumerable<Point3F> points, int normalK, int featureK, int threadsCount);
        #endregion

        #region # 计算FPFH特征 —— FPFHSignature33F[] ComputeFPFH(IEnumerable<Point3F> points, int normalK...
        /// <summary>
        /// 计算FPFH特征
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="featureK">特征K</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>FPFH特征描述子集</returns>
        /// <remarks>特征K值要大于法向量K值</remarks>
        FPFHSignature33F[] ComputeFPFH(IEnumerable<Point3F> points, int normalK, int featureK, int threadsCount);
        #endregion

        #region # 计算3DSC特征 —— ShapeContext1980F[] Compute3DSC(IEnumerable<Point3F> points, int normalK...
        /// <summary>
        /// 计算3DSC特征
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="searchRadius">搜索半径</param>
        /// <param name="pointDensityRadius">点密度半径</param>
        /// <param name="minimalRadius">最小半径</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>3DSC特征描述子集</returns>
        /// <remarks>
        /// 点密度半径建议设置为搜索半径的1/5；
        ///	最小半径建议设置为搜索半径的1/10；
        /// </remarks>
        ShapeContext1980F[] Compute3DSC(IEnumerable<Point3F> points, int normalK, float searchRadius, float pointDensityRadius, float minimalRadius, int threadsCount);
        #endregion

        #region # 计算SHOT特征 —— Shot352F[] ComputeSHOT(IEnumerable<Point3F> points, int normalK...
        /// <summary>
        /// 计算SHOT特征
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="normalK">法向量K</param>
        /// <param name="featureRadius">特征搜索半径</param>
        /// <param name="threadsCount">线程数</param>
        /// <returns>SHOT特征描述子集</returns>
        Shot352F[] ComputeSHOT(IEnumerable<Point3F> points, int normalK, float featureRadius, int threadsCount);
        #endregion
    }
}
