using PCLSharp.Primitives.Constants;
using SD.Common;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
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
            this.Reset();
        }

        #endregion

        #region # 属性

        //一次采样

        #region 是否一次采样 —— bool NeedSampleI
        /// <summary>
        /// 是否一次采样
        /// </summary>
        [DependencyProperty]
        public bool NeedSampleI { get; set; }
        #endregion

        #region 一次采样可用性 —— bool SampleIEnabled
        /// <summary>
        /// 一次采样可用性
        /// </summary>
        [DependencyProperty]
        public bool SampleIEnabled { get; set; }
        #endregion

        #region 一次网格尺寸 —— float? SampleILeafSize
        /// <summary>
        /// 一次网格尺寸
        /// </summary>
        [DependencyProperty]
        public float? SampleILeafSize { get; set; }
        #endregion


        //粗配准

        #region 是否粗配准 —— bool NeedCoarseAlignment
        /// <summary>
        /// 是否粗配准
        /// </summary>
        [DependencyProperty]
        public bool NeedCoarseAlignment { get; set; }
        #endregion

        #region 粗配准可用性 —— bool CoarseAlignmentEnabled
        /// <summary>
        /// 粗配准可用性
        /// </summary>
        [DependencyProperty]
        public bool CoarseAlignmentEnabled { get; set; }
        #endregion

        #region 粗配准模式 —— CoarseAlignmentMode? SelectedCoarseAlignmentMode
        /// <summary>
        /// 粗配准模式
        /// </summary>
        [DependencyProperty]
        public CoarseAlignmentMode? SelectedCoarseAlignmentMode { get; set; }
        #endregion

        #region 采样数量 —— int? SamplesCount
        /// <summary>
        /// 采样数量
        /// </summary>
        [DependencyProperty]
        public int? SamplesCount { get; set; }
        #endregion

        #region 是否分割 —— bool? NeedToSegment
        /// <summary>
        /// 是否分割
        /// </summary>
        [DependencyProperty]
        public bool? NeedToSegment { get; set; }
        #endregion

        #region 是否离群点移除 —— bool? NeedOutlierRemoval
        /// <summary>
        /// 是否离群点移除
        /// </summary>
        [DependencyProperty]
        public bool? NeedOutlierRemoval { get; set; }
        #endregion

        #region 粗配准模式字典 —— IDictionary<string, string> CoarseAlignmentModes
        /// <summary>
        /// 粗配准模式字典
        /// </summary>
        [DependencyProperty]
        public IDictionary<string, string> CoarseAlignmentModes { get; set; }
        #endregion


        //粗配准: K-FPCS

        #region K-FPCS可用性 —— bool KFPCSEnabled
        /// <summary>
        /// K-FPCS可用性
        /// </summary>
        [DependencyProperty]
        public bool KFPCSEnabled { get; set; }
        #endregion

        #region 近似重叠 —— float? ApproxOverlap
        /// <summary>
        /// 近似重叠
        /// </summary>
        [DependencyProperty]
        public float? ApproxOverlap { get; set; }
        #endregion

        #region 平移向量系数 —— float? Lambda
        /// <summary>
        /// 平移向量系数
        /// </summary>
        [DependencyProperty]
        public float? Lambda { get; set; }
        #endregion

        #region 配准距离 —— float? Delta
        /// <summary>
        /// 配准距离
        /// </summary>
        [DependencyProperty]
        public float? Delta { get; set; }
        #endregion

        #region 是否标准化 —— bool? NeedToNormalize
        /// <summary>
        /// 是否标准化
        /// </summary>
        [DependencyProperty]
        public bool? NeedToNormalize { get; set; }
        #endregion

        #region 最大计算时间 —— int? MaxComputationTime
        /// <summary>
        /// 最大计算时间
        /// </summary>
        /// <remarks>单位：秒</remarks>
        [DependencyProperty]
        public int? MaxComputationTime { get; set; }
        #endregion


        //粗配准: SAC-IA

        #region SAC-IA可用性 —— bool SACIAEnabled
        /// <summary>
        /// SAC-IA可用性
        /// </summary>
        [DependencyProperty]
        public bool SACIAEnabled { get; set; }
        #endregion

        #region 采样点最小距离 —— float? MinSampleDistance
        /// <summary>
        /// 采样点最小距离
        /// </summary>
        [DependencyProperty]
        public float? MinSampleDistance { get; set; }
        #endregion

        #region 随机特征邻域点数 —— int? CorrespondenceRandomness
        /// <summary>
        /// 随机特征邻域点数
        /// </summary>
        [DependencyProperty]
        public int? CorrespondenceRandomness { get; set; }
        #endregion


        //粗配准-分割

        #region 分割可用性 —— bool SegmentEnabled
        /// <summary>
        /// 分割可用性
        /// </summary>
        [DependencyProperty]
        public bool SegmentEnabled { get; set; }
        #endregion

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


        //粗配准-离群点移除

        #region 离群点移除可用性 —— bool OutlierRemovalEnabled
        /// <summary>
        /// 离群点移除可用性
        /// </summary>
        [DependencyProperty]
        public bool OutlierRemovalEnabled { get; set; }
        #endregion

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


        //粗配准: SAC-IA-关键点

        #region 关键点可用性 —— bool KeyPointEnabled
        /// <summary>
        /// 关键点可用性
        /// </summary>
        [DependencyProperty]
        public bool KeyPointEnabled { get; set; }
        #endregion

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


        //粗配准: SAC-IA-特征

        #region 特征可用性 —— bool FeatureEnabled
        /// <summary>
        /// 特征可用性
        /// </summary>
        [DependencyProperty]
        public bool FeatureEnabled { get; set; }
        #endregion

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


        //二次采样

        #region 是否二次采样 —— bool NeedSampleII
        /// <summary>
        /// 是否二次采样
        /// </summary>
        [DependencyProperty]
        public bool NeedSampleII { get; set; }
        #endregion

        #region 二次采样可用性 —— bool SampleIIEnabled
        /// <summary>
        /// 二次采样可用性
        /// </summary>
        [DependencyProperty]
        public bool SampleIIEnabled { get; set; }
        #endregion

        #region 二次网格尺寸 —— float? SampleIILeafSize
        /// <summary>
        /// 二次网格尺寸
        /// </summary>
        [DependencyProperty]
        public float? SampleIILeafSize { get; set; }
        #endregion


        //精配准

        #region 是否精配准 —— bool NeedFineAlignment
        /// <summary>
        /// 是否精配准
        /// </summary>
        [DependencyProperty]
        public bool NeedFineAlignment { get; set; }
        #endregion

        #region 精配准可用性 —— bool FineAlignmentEnabled
        /// <summary>
        /// 精配准可用性
        /// </summary>
        [DependencyProperty]
        public bool FineAlignmentEnabled { get; set; }
        #endregion

        #region 精配准模式 —— FineAlignmentMode? SelectedFineAlignmentMode
        /// <summary>
        /// 精配准模式
        /// </summary>
        [DependencyProperty]
        public FineAlignmentMode? SelectedFineAlignmentMode { get; set; }
        #endregion

        #region 变换最大差值 —— float? TransformationEpsilon
        /// <summary>
        /// 变换最大差值
        /// </summary>
        [DependencyProperty]
        public float? TransformationEpsilon { get; set; }
        #endregion

        #region 最大迭代次数 —— int? MaximumIterations
        /// <summary>
        /// 最大迭代次数
        /// </summary>
        [DependencyProperty]
        public int? MaximumIterations { get; set; }
        #endregion

        #region 精配准模式字典 —— IDictionary<string, string> FineAlignmentModes
        /// <summary>
        /// 精配准模式字典
        /// </summary>
        [DependencyProperty]
        public IDictionary<string, string> FineAlignmentModes { get; set; }
        #endregion


        //精配准: ICP

        #region ICP可用性 —— bool ICPEnabled
        /// <summary>
        /// ICP可用性
        /// </summary>
        [DependencyProperty]
        public bool ICPEnabled { get; set; }
        #endregion

        #region 最大相似距离 —— float? MaxCorrespondenceDistance
        /// <summary>
        /// 最大相似距离
        /// </summary>
        [DependencyProperty]
        public float? MaxCorrespondenceDistance { get; set; }
        #endregion

        #region 均方误差阈值 —— float? EuclideanFitnessEpsilon
        /// <summary>
        /// 均方误差阈值
        /// </summary>
        [DependencyProperty]
        public float? EuclideanFitnessEpsilon { get; set; }
        #endregion


        //精配准: NDT

        #region NDT可用性 —— bool NDTEnabled
        /// <summary>
        /// NDT可用性
        /// </summary>
        [DependencyProperty]
        public bool NDTEnabled { get; set; }
        #endregion

        #region 分辨率 —— float? Resolution
        /// <summary>
        /// 分辨率
        /// </summary>
        [DependencyProperty]
        public float? Resolution { get; set; }
        #endregion

        #region 步长 —— float? StepSize
        /// <summary>
        /// 步长
        /// </summary>
        [DependencyProperty]
        public float? StepSize { get; set; }
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

        //Initializations

        #region 初始化 —— override Task OnInitializeAsync(CancellationToken cancellationToken)
        /// <summary>
        /// 初始化
        /// </summary>
        protected override Task OnInitializeAsync(CancellationToken cancellationToken)
        {
            this.CoarseAlignmentModes = typeof(CoarseAlignmentMode).GetEnumMembers();
            this.FineAlignmentModes = typeof(FineAlignmentMode).GetEnumMembers();

            return base.OnInitializeAsync(cancellationToken);
        }
        #endregion


        //Actions

        #region 切换一次采样 —— void SwitchSampleI()
        /// <summary>
        /// 切换一次采样
        /// </summary>
        public void SwitchSampleI()
        {
            this.SampleIEnabled = this.NeedSampleI;
        }
        #endregion

        #region 切换粗配准 —— void SwitchCoarseAlignment()
        /// <summary>
        /// 切换粗配准
        /// </summary>
        public void SwitchCoarseAlignment()
        {
            if (this.NeedCoarseAlignment)
            {
                this.CoarseAlignmentEnabled = true;
            }
            else
            {
                this.CoarseAlignmentEnabled = false;
                this.KFPCSEnabled = false;
                this.SACIAEnabled = false;
                this.NeedToSegment = false;
                this.SegmentEnabled = false;
                this.NeedOutlierRemoval = false;
                this.OutlierRemovalEnabled = false;
                this.SelectedCoarseAlignmentMode = null;
            }
        }
        #endregion

        #region 切换粗配准模式 —— void SwitchCoarseAlignmentMode()
        /// <summary>
        /// 切换粗配准模式
        /// </summary>
        public void SwitchCoarseAlignmentMode()
        {
            if (this.SelectedCoarseAlignmentMode == CoarseAlignmentMode.KFPCS)
            {
                this.KFPCSEnabled = true;
                this.SACIAEnabled = false;
                this.KeyPointEnabled = false;
                this.FeatureEnabled = false;
            }
            else if (this.SelectedCoarseAlignmentMode == CoarseAlignmentMode.SACIA)
            {
                this.SACIAEnabled = true;
                this.KFPCSEnabled = false;
                this.KeyPointEnabled = true;
                this.FeatureEnabled = true;
            }
            else
            {
                this.KFPCSEnabled = false;
                this.SACIAEnabled = false;
                this.KeyPointEnabled = false;
                this.FeatureEnabled = false;
            }
        }
        #endregion

        #region 切换是否分割 —— void SwitchNeedToSegment()
        /// <summary>
        /// 切换是否分割
        /// </summary>
        public void SwitchNeedToSegment()
        {
            this.SegmentEnabled = this.NeedToSegment == true;
        }
        #endregion

        #region 切换是否离群点移除 —— void SwitchNeedOutlierRemoval()
        /// <summary>
        /// 切换是否离群点移除
        /// </summary>
        public void SwitchNeedOutlierRemoval()
        {
            this.OutlierRemovalEnabled = this.NeedOutlierRemoval == true;
        }
        #endregion

        #region 切换二次采样 —— void SwitchSampleII()
        /// <summary>
        /// 切换二次采样
        /// </summary>
        public void SwitchSampleII()
        {
            this.SampleIIEnabled = this.NeedSampleII;
        }
        #endregion

        #region 切换精配准 —— void SwitchFineAlignment()
        /// <summary>
        /// 切换精配准
        /// </summary>
        public void SwitchFineAlignment()
        {
            if (this.NeedFineAlignment)
            {
                this.FineAlignmentEnabled = true;
            }
            else
            {
                this.FineAlignmentEnabled = false;
                this.ICPEnabled = false;
                this.NDTEnabled = false;
                this.SelectedFineAlignmentMode = null;
            }
        }
        #endregion

        #region 切换精配准模式 —— void SwitchFineAlignmentMode()
        /// <summary>
        /// 切换精配准模式
        /// </summary>
        public void SwitchFineAlignmentMode()
        {
            if (this.SelectedFineAlignmentMode == FineAlignmentMode.PointToPoint ||
                this.SelectedFineAlignmentMode == FineAlignmentMode.PointToPlane ||
                this.SelectedFineAlignmentMode == FineAlignmentMode.GICP)
            {
                this.ICPEnabled = true;
                this.NDTEnabled = false;
            }
            else if (this.SelectedFineAlignmentMode == FineAlignmentMode.NDT)
            {
                this.NDTEnabled = true;
                this.ICPEnabled = false;
            }
            else
            {
                this.ICPEnabled = false;
                this.NDTEnabled = false;
            }
        }
        #endregion

        #region 重置 —— void Reset()
        /// <summary>
        /// 重置
        /// </summary>
        public void Reset()
        {
            //主参数
            this.NeedSampleI = true;
            this.SampleIEnabled = true;
            this.SampleILeafSize = 1.0f;
            this.NeedCoarseAlignment = true;
            this.NeedSampleII = true;
            this.SampleIIEnabled = true;
            this.SampleIILeafSize = 2.0f;
            this.NeedFineAlignment = true;
            this.ThreadsCount = 20;

            //粗配准
            this.CoarseAlignmentEnabled = true;
            this.SelectedCoarseAlignmentMode = CoarseAlignmentMode.SACIA;
            this.SamplesCount = 100;
            this.NeedToSegment = true;
            this.SegmentEnabled = true;
            this.NeedOutlierRemoval = true;
            this.OutlierRemovalEnabled = true;

            //精配准
            this.FineAlignmentEnabled = true;
            this.SelectedFineAlignmentMode = FineAlignmentMode.PointToPlane;
            this.TransformationEpsilon = 1e-6f;
            this.MaximumIterations = 35;

            //K-FPCS
            this.KFPCSEnabled = false;
            this.ApproxOverlap = 0.7f;
            this.Lambda = 1.5f;
            this.Delta = 0.002f;
            this.NeedToNormalize = false;
            this.MaxComputationTime = 1000;

            //SAC-IA
            this.SACIAEnabled = true;
            this.MinSampleDistance = 7.0f;
            this.CorrespondenceRandomness = 6;

            //分割
            this.SegmentEnabled = true;
            this.ClusterTolerance = 1.5f;
            this.MinClusterSize = 1000;
            this.MaxClusterSize = 100000;

            //离群点移除
            this.OutlierRemovalEnabled = true;
            this.MeanK = 50;
            this.StddevMult = 1.0f;

            //关键点
            this.KeyPointEnabled = true;
            this.SalientRadius = 5.0f;
            this.NonMaxRadius = 5.0f;
            this.Threshold21 = 0.95f;
            this.Threshold32 = 0.95f;
            this.MinNeighborsCount = 6;

            //特征
            this.FeatureEnabled = true;
            this.NormalK = 10;
            this.FeatureK = 15;

            //ICP
            this.ICPEnabled = true;
            this.MaxCorrespondenceDistance = 100.0f;
            this.EuclideanFitnessEpsilon = 0.1f;

            //NDT
            this.NDTEnabled = false;
            this.Resolution = 1.5f;
            this.StepSize = 0.1f;
        }
        #endregion

        #region 提交 —— async void Submit()
        /// <summary>
        /// 提交
        /// </summary>
        public async void Submit()
        {
            #region # 验证

            if (!this.CoarseAlignmentEnabled && !this.FineAlignmentEnabled)
            {
                MessageBox.Show("粗配准与精配准不可同时关闭！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ThreadsCount.HasValue)
            {
                MessageBox.Show("线程数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //一次采样
            if (this.SampleIEnabled && !this.SampleILeafSize.HasValue)
            {
                MessageBox.Show("一次网格尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //粗配准
            if (this.CoarseAlignmentEnabled && !this.SelectedCoarseAlignmentMode.HasValue)
            {
                MessageBox.Show("粗配准模式不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.CoarseAlignmentEnabled && !this.SamplesCount.HasValue)
            {
                MessageBox.Show("采样数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.CoarseAlignmentEnabled && !this.NeedToSegment.HasValue)
            {
                MessageBox.Show("是否分割不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.CoarseAlignmentEnabled && !this.NeedOutlierRemoval.HasValue)
            {
                MessageBox.Show("是否离群点移除不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //粗配准: K-FPCS
            if (this.KFPCSEnabled && !this.ApproxOverlap.HasValue)
            {
                MessageBox.Show("近似重叠不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.KFPCSEnabled && !this.Lambda.HasValue)
            {
                MessageBox.Show("平移向量系数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.KFPCSEnabled && !this.Delta.HasValue)
            {
                MessageBox.Show("配准距离不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.KFPCSEnabled && !this.NeedToNormalize.HasValue)
            {
                MessageBox.Show("是否标准化不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.KFPCSEnabled && !this.MaxComputationTime.HasValue)
            {
                MessageBox.Show("最大计算时间不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //粗配准: SAC-IA
            if (this.SACIAEnabled && !this.MinSampleDistance.HasValue)
            {
                MessageBox.Show("采样点最小距离不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.SACIAEnabled && !this.CorrespondenceRandomness.HasValue)
            {
                MessageBox.Show("随机特征邻域点数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //粗配准-分割
            if (this.SegmentEnabled && !this.ClusterTolerance.HasValue)
            {
                MessageBox.Show("簇搜索容差不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.SegmentEnabled && !this.MinClusterSize.HasValue)
            {
                MessageBox.Show("簇最小尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.SegmentEnabled && !this.MaxClusterSize.HasValue)
            {
                MessageBox.Show("簇最大尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //粗配准-离群点移除
            if (this.OutlierRemovalEnabled && !this.MeanK.HasValue)
            {
                MessageBox.Show("平均近邻K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.OutlierRemovalEnabled && !this.StddevMult.HasValue)
            {
                MessageBox.Show("标准差系数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //粗配准: SAC-IA-关键点
            if (this.KeyPointEnabled && !this.SalientRadius.HasValue)
            {
                MessageBox.Show("显著半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.KeyPointEnabled && !this.NonMaxRadius.HasValue)
            {
                MessageBox.Show("抑制半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.KeyPointEnabled && !this.Threshold21.HasValue)
            {
                MessageBox.Show("二一比上限不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.KeyPointEnabled && !this.Threshold32.HasValue)
            {
                MessageBox.Show("三二比上限不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.KeyPointEnabled && !this.MinNeighborsCount.HasValue)
            {
                MessageBox.Show("最小邻域点数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //粗配准: SAC-IA-特征
            if (this.FeatureEnabled && !this.NormalK.HasValue)
            {
                MessageBox.Show("法向量K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.FeatureEnabled && !this.FeatureK.HasValue)
            {
                MessageBox.Show("特征K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //二次采样
            if (this.SampleIIEnabled && !this.SampleIILeafSize.HasValue)
            {
                MessageBox.Show("二次网格尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //精配准
            if (this.FineAlignmentEnabled && !this.SelectedFineAlignmentMode.HasValue)
            {
                MessageBox.Show("精配准模式不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.FineAlignmentEnabled && !this.TransformationEpsilon.HasValue)
            {
                MessageBox.Show("变换最大差值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.FineAlignmentEnabled && !this.MaximumIterations.HasValue)
            {
                MessageBox.Show("最大迭代次数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //精配准: ICP
            if (this.ICPEnabled && !this.MaxCorrespondenceDistance.HasValue)
            {
                MessageBox.Show("最大相似距离不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.ICPEnabled && !this.EuclideanFitnessEpsilon.HasValue)
            {
                MessageBox.Show("均方误差阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            //精配准: NDT
            if (this.NDTEnabled && !this.Resolution.HasValue)
            {
                MessageBox.Show("网格分辨率不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.NDTEnabled && !this.StepSize.HasValue)
            {
                MessageBox.Show("最大步长不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
