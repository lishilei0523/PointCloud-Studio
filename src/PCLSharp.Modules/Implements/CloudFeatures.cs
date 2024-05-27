using PCLSharp.Modules.Declarations;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Features;
using PCLSharp.Primitives.Models;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;

namespace PCLSharp.Modules.Implements
{
    /// <summary>
    /// 点云特征实现
    /// </summary>
    public class CloudFeatures : ICloudFeatures
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
        public Narf36F[] ComputeNARF(IEnumerable<Point3F> points, float angularResolution, float maxAngleWidth, float maxAngleHeight, float noiseLevel, float minRange, int borderSize, float supportSize, bool rotationInvariant)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Narf36F>();
            }

            #endregion

            IntPtr pointer = FeaturesNative.ComputeNARF(points_, points_.Length, angularResolution, maxAngleWidth, maxAngleHeight, noiseLevel, minRange, borderSize, supportSize, rotationInvariant);
            Narf36Fs narf36Fs = Marshal.PtrToStructure<Narf36Fs>(pointer);
            Narf36F[] descriptors = narf36Fs.Recover();
            DisposeNative.DisposeNarf36Fs(pointer);

            return descriptors;
        }
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
        public PFHSignature125F[] ComputePFH(IEnumerable<Point3F> points, int normalK, int featureK, int threadsCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<PFHSignature125F>();
            }

            #endregion

            IntPtr pointer = FeaturesNative.ComputePFH(points_, points_.Length, normalK, featureK, threadsCount);
            PFHSignature125Fs signature125Fs = Marshal.PtrToStructure<PFHSignature125Fs>(pointer);
            PFHSignature125F[] descriptors = signature125Fs.Recover();
            DisposeNative.DisposePFHSignature125Fs(pointer);

            return descriptors;
        }
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
        public FPFHSignature33F[] ComputeFPFH(IEnumerable<Point3F> points, int normalK, int featureK, int threadsCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<FPFHSignature33F>();
            }

            #endregion

            IntPtr pointer = FeaturesNative.ComputeFPFH(points_, points_.Length, normalK, featureK, threadsCount);
            FPFHSignature33Fs signature33Fs = Marshal.PtrToStructure<FPFHSignature33Fs>(pointer);
            FPFHSignature33F[] descriptors = signature33Fs.Recover();
            DisposeNative.DisposeFPFHSignature33Fs(pointer);

            return descriptors;
        }
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
        public ShapeContext1980F[] Compute3DSC(IEnumerable<Point3F> points, int normalK, float searchRadius, float pointDensityRadius, float minimalRadius, int threadsCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<ShapeContext1980F>();
            }

            #endregion

            IntPtr pointer = FeaturesNative.Compute3DSC(points_, points_.Length, normalK, searchRadius, pointDensityRadius, minimalRadius, threadsCount);
            ShapeContext1980Fs shapeContext1980Fs = Marshal.PtrToStructure<ShapeContext1980Fs>(pointer);
            ShapeContext1980F[] descriptors = shapeContext1980Fs.Recover();
            DisposeNative.DisposeShapeContext1980Fs(pointer);

            return descriptors;
        }
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
        public Shot352F[] ComputeSHOT(IEnumerable<Point3F> points, int normalK, float featureRadius, int threadsCount)
        {
            Point3F[] points_ = points?.ToArray() ?? Array.Empty<Point3F>();

            #region # 验证

            if (!points_.Any())
            {
                return Array.Empty<Shot352F>();
            }

            #endregion

            IntPtr pointer = FeaturesNative.ComputeSHOT(points_, points_.Length, normalK, featureRadius, threadsCount);
            Shot352Fs shot352Fs = Marshal.PtrToStructure<Shot352Fs>(pointer);
            Shot352F[] descriptors = shot352Fs.Recover();
            DisposeNative.DisposeShot352Fs(pointer);

            return descriptors;
        }
        #endregion
    }
}
