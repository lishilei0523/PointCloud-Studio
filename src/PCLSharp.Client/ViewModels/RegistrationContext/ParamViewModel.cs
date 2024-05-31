using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.RegistrationContext
{
    /// <summary>
    /// 点云配准参数视图模型
    /// </summary>
    public class ParamViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public ParamViewModel()
        {
            //降采样
            this.LeafSize = 0.01f;

            //分割
            this.ClusterTolerance = 1.5f;
            this.MinClusterSize = 1000;
            this.MaxClusterSize = 100000;

            //离群点移除
            this.MeanK = 50;
            this.StddevMult = 1.0f;

            //关键点
            this.SalientRadius = 5.0f;
            this.NonMaxRadius = 5.0f;
            this.Threshold21 = 0.95f;
            this.Threshold32 = 0.95f;
            this.MinNeighborsCount = 6;

            //特征
            this.NormalK = 10;
            this.FeatureK = 15;

            //粗配准
            this.MinSampleDistance = 7.0f;
            this.SamplesCount = 100;
            this.CorrespondenceRandomness = 6;

            //精配准
            this.MaxCorrespondenceDistance = 100.0f;
            this.TransformationEpsilon = 1e-6f;
            this.EuclideanFitnessEpsilon = 0.1f;
            this.MaximumIterations = 35;

            //其他
            this.ThreadsCount = 20;
        }

        #endregion

        #region # 属性

        //降采样

        #region 叶子尺寸 —— float? LeafSize
        /// <summary>
        /// 叶子尺寸
        /// </summary>
        [DependencyProperty]
        public float? LeafSize { get; set; }
        #endregion


        //分割

        #region 簇搜索容差 —— float? ClusterTolerance
        /// <summary>
        /// 簇搜索容差
        /// </summary>
        [DependencyProperty]
        public float? ClusterTolerance { get; set; }
        #endregion

        #region 簇最小尺寸 —— int? MinClusterSize
        /// <summary>
        /// 簇最小尺寸
        /// </summary>
        [DependencyProperty]
        public int? MinClusterSize { get; set; }
        #endregion

        #region 簇最大尺寸 —— int? MaxClusterSize
        /// <summary>
        /// 簇最大尺寸
        /// </summary>
        [DependencyProperty]
        public int? MaxClusterSize { get; set; }
        #endregion


        //离群点移除

        #region 平均近邻K —— int? MeanK
        /// <summary>
        /// 平均近邻K
        /// </summary>
        [DependencyProperty]
        public int? MeanK { get; set; }
        #endregion

        #region 标准差系数 —— float? StddevMult
        /// <summary>
        /// 标准差系数
        /// </summary>
        [DependencyProperty]
        public float? StddevMult { get; set; }
        #endregion


        //关键点

        #region 显著半径 —— float? SalientRadius
        /// <summary>
        /// 显著半径
        /// </summary>
        [DependencyProperty]
        public float? SalientRadius { get; set; }
        #endregion

        #region 抑制半径 —— float? NonMaxRadius
        /// <summary>
        /// 抑制半径
        /// </summary>
        [DependencyProperty]
        public float? NonMaxRadius { get; set; }
        #endregion

        #region 二一比上限 —— float? Threshold21
        /// <summary>
        /// 二一比上限
        /// </summary>
        [DependencyProperty]
        public float? Threshold21 { get; set; }
        #endregion

        #region 三二比上限 —— float? Threshold32
        /// <summary>
        /// 三二比上限
        /// </summary>
        [DependencyProperty]
        public float? Threshold32 { get; set; }
        #endregion

        #region 最小邻域点数 —— int? MinNeighborsCount
        /// <summary>
        /// 最小邻域点数
        /// </summary>
        [DependencyProperty]
        public int? MinNeighborsCount { get; set; }
        #endregion


        //特征

        #region 法向量K —— int? NormalK
        /// <summary>
        /// 法向量K
        /// </summary>
        [DependencyProperty]
        public int? NormalK { get; set; }
        #endregion

        #region 特征K —— int? FeatureK
        /// <summary>
        /// 特征K
        /// </summary>
        [DependencyProperty]
        public int? FeatureK { get; set; }
        #endregion


        //粗配准

        #region 采样点最小距离 —— float? MinSampleDistance
        /// <summary>
        /// 采样点最小距离
        /// </summary>
        [DependencyProperty]
        public float? MinSampleDistance { get; set; }
        #endregion

        #region 采样数量 —— int? SamplesCount
        /// <summary>
        /// 采样数量
        /// </summary>
        [DependencyProperty]
        public int? SamplesCount { get; set; }
        #endregion

        #region 随机特征邻域点数 —— int? CorrespondenceRandomness
        /// <summary>
        /// 随机特征邻域点数
        /// </summary>
        [DependencyProperty]
        public int? CorrespondenceRandomness { get; set; }
        #endregion


        //精配准

        #region 分辨率 —— float? MaxCorrespondenceDistance
        /// <summary>
        /// 分辨率
        /// </summary>
        [DependencyProperty]
        public float? MaxCorrespondenceDistance { get; set; }
        #endregion

        #region 变换最大差值 —— float? TransformationEpsilon
        /// <summary>
        /// 变换最大差值
        /// </summary>
        [DependencyProperty]
        public float? TransformationEpsilon { get; set; }
        #endregion

        #region 均方误差阈值 —— float? EuclideanFitnessEpsilon
        /// <summary>
        /// 均方误差阈值
        /// </summary>
        [DependencyProperty]
        public float? EuclideanFitnessEpsilon { get; set; }
        #endregion

        #region 最大迭代次数 —— int? MaximumIterations
        /// <summary>
        /// 最大迭代次数
        /// </summary>
        [DependencyProperty]
        public int? MaximumIterations { get; set; }
        #endregion


        //其他

        #region 线程数 —— int? ThreadsCount
        /// <summary>
        /// 线程数
        /// </summary>
        [DependencyProperty]
        public int? ThreadsCount { get; set; }
        #endregion

        #endregion

        #region # 方法

        #region 提交 —— async void Submit()
        /// <summary>
        /// 提交
        /// </summary>
        public async void Submit()
        {
            #region # 验证

            //降采样
            if (!this.LeafSize.HasValue)
            {
                MessageBox.Show("叶子尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //分割
            if (!this.ClusterTolerance.HasValue)
            {
                MessageBox.Show("簇搜索容差不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinClusterSize.HasValue)
            {
                MessageBox.Show("簇最小尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxClusterSize.HasValue)
            {
                MessageBox.Show("簇最大尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //离群点移除
            if (!this.MeanK.HasValue)
            {
                MessageBox.Show("平均距离估计的最近邻居的数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.StddevMult.HasValue)
            {
                MessageBox.Show("标准差阈值系数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //关键点
            if (!this.SalientRadius.HasValue)
            {
                MessageBox.Show("显著半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.NonMaxRadius.HasValue)
            {
                MessageBox.Show("非极大值抑制半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Threshold21.HasValue)
            {
                MessageBox.Show("二一特征值比上限不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Threshold32.HasValue)
            {
                MessageBox.Show("三二特征值比上限不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinNeighborsCount.HasValue)
            {
                MessageBox.Show("最小邻域点数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //特征
            if (!this.NormalK.HasValue)
            {
                MessageBox.Show("法向量K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.FeatureK.HasValue)
            {
                MessageBox.Show("特征K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //粗配准
            if (!this.MinSampleDistance.HasValue)
            {
                MessageBox.Show("采样点最小距离不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.SamplesCount.HasValue)
            {
                MessageBox.Show("采样数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.CorrespondenceRandomness.HasValue)
            {
                MessageBox.Show("随机特征邻域点数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //精配准
            if (!this.MaxCorrespondenceDistance.HasValue)
            {
                MessageBox.Show("分辨率不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.TransformationEpsilon.HasValue)
            {
                MessageBox.Show("变换最大差值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.EuclideanFitnessEpsilon.HasValue)
            {
                MessageBox.Show("均方误差阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaximumIterations.HasValue)
            {
                MessageBox.Show("最大迭代次数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //其他
            if (!this.ThreadsCount.HasValue)
            {
                MessageBox.Show("线程数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
