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
    public class Param2ViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public Param2ViewModel()
        {
            this.NeedSampleI = false;
            this.NeedSampleII = false;
            this.SampleIVisibility = Visibility.Collapsed;
            this.SampleIIVisibility = Visibility.Collapsed;

            this.NeedCoarseAlignment = false;
            this.NeedFineAlignment = false;
            this.CoarseAlignmentVisibility = Visibility.Collapsed;
            this.FineAlignmentVisibility = Visibility.Collapsed;
            this.KFPCSVisibility = Visibility.Collapsed;
            this.SACIAVisibility = Visibility.Collapsed;

            //降采样
            this.SampleILeafSize = 1.0f;
            this.SampleIILeafSize = 2.0f;

            //分割
            this.NeedToSegment = true;
            this.ClusterTolerance = 1.5f;
            this.MinClusterSize = 1000;
            this.MaxClusterSize = 100000;

            //离群点移除
            this.NeedOutlierRemoval = true;
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

        //一次采样

        #region 是否一次采样 —— bool NeedSampleI
        /// <summary>
        /// 是否一次采样
        /// </summary>
        [DependencyProperty]
        public bool NeedSampleI { get; set; }
        #endregion

        #region 一次采样可见性 —— Visibility SampleIVisibility
        /// <summary>
        /// 一次采样可见性
        /// </summary>
        [DependencyProperty]
        public Visibility SampleIVisibility { get; set; }
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

        #region 粗配准可见性 —— Visibility CoarseAlignmentVisibility
        /// <summary>
        /// 粗配准可见性
        /// </summary>
        [DependencyProperty]
        public Visibility CoarseAlignmentVisibility { get; set; }
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

        #region K-FPCS可见性 —— Visibility KFPCSVisibility
        /// <summary>
        /// K-FPCS可见性
        /// </summary>
        [DependencyProperty]
        public Visibility KFPCSVisibility { get; set; }
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

        #region 最大计算时间 —— int? MaxComputationTime
        /// <summary>
        /// 最大计算时间
        /// </summary>
        /// <remarks>单位：秒</remarks>
        [DependencyProperty]
        public int? MaxComputationTime { get; set; }
        #endregion


        //粗配准: SAC-IA

        #region SAC-IA可见性 —— Visibility SACIAVisibility
        /// <summary>
        /// SAC-IA可见性
        /// </summary>
        [DependencyProperty]
        public Visibility SACIAVisibility { get; set; }
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

        #region 分割可见性 —— Visibility SegmentVisibility
        /// <summary>
        /// 分割可见性
        /// </summary>
        [DependencyProperty]
        public Visibility SegmentVisibility { get; set; }
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

        #region 离群点移除可见性 —— Visibility OutlierRemovalVisibility
        /// <summary>
        /// 离群点移除可见性
        /// </summary>
        [DependencyProperty]
        public Visibility OutlierRemovalVisibility { get; set; }
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

        #region 关键点可见性 —— Visibility KeyPointVisibility
        /// <summary>
        /// 关键点可见性
        /// </summary>
        [DependencyProperty]
        public Visibility KeyPointVisibility { get; set; }
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

        #region 特征可见性 —— Visibility FeatureVisibility
        /// <summary>
        /// 特征可见性
        /// </summary>
        [DependencyProperty]
        public Visibility FeatureVisibility { get; set; }
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

        #region 二次采样可见性 —— Visibility SampleIIVisibility
        /// <summary>
        /// 二次采样可见性
        /// </summary>
        [DependencyProperty]
        public Visibility SampleIIVisibility { get; set; }
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

        #region 精配准可见性 —— Visibility FineAlignmentVisibility
        /// <summary>
        /// 精配准可见性
        /// </summary>
        [DependencyProperty]
        public Visibility FineAlignmentVisibility { get; set; }
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

        #region ICP可见性 —— Visibility ICPVisibility
        /// <summary>
        /// ICP可见性
        /// </summary>
        [DependencyProperty]
        public Visibility ICPVisibility { get; set; }
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

        #region NDT可见性 —— Visibility NDTVisibility
        /// <summary>
        /// NDT可见性
        /// </summary>
        [DependencyProperty]
        public Visibility NDTVisibility { get; set; }
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
            this.SampleIVisibility = this.NeedSampleI ? Visibility.Visible : Visibility.Collapsed;
        }
        #endregion

        #region 切换粗配准 —— void SwitchCoarseAlignment()
        /// <summary>
        /// 切换粗配准
        /// </summary>
        public void SwitchCoarseAlignment()
        {
            this.CoarseAlignmentVisibility = this.NeedCoarseAlignment ? Visibility.Visible : Visibility.Collapsed;
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
                this.KFPCSVisibility = Visibility.Visible;
                this.SACIAVisibility = Visibility.Collapsed;
            }
            else if (this.SelectedCoarseAlignmentMode == CoarseAlignmentMode.SACIA)
            {
                this.SACIAVisibility = Visibility.Visible;
                this.KFPCSVisibility = Visibility.Collapsed;
            }
            else
            {
                this.KFPCSVisibility = Visibility.Collapsed;
                this.SACIAVisibility = Visibility.Collapsed;
            }
        }
        #endregion

        #region 切换是否分割 —— void SwitchNeedToSegment()
        /// <summary>
        /// 切换是否分割
        /// </summary>
        public void SwitchNeedToSegment()
        {
            this.SegmentVisibility = this.NeedToSegment == true ? Visibility.Visible : Visibility.Collapsed;
        }
        #endregion

        #region 切换是否离群点移除 —— void SwitchNeedOutlierRemoval()
        /// <summary>
        /// 切换是否离群点移除
        /// </summary>
        public void SwitchNeedOutlierRemoval()
        {
            this.OutlierRemovalVisibility = this.NeedOutlierRemoval == true ? Visibility.Visible : Visibility.Collapsed;
        }
        #endregion

        #region 切换二次采样 —— void SwitchSampleII()
        /// <summary>
        /// 切换二次采样
        /// </summary>
        public void SwitchSampleII()
        {
            this.SampleIIVisibility = this.NeedSampleII ? Visibility.Visible : Visibility.Collapsed;
        }
        #endregion

        #region 切换精配准 —— void SwitchFineAlignment()
        /// <summary>
        /// 切换精配准
        /// </summary>
        public void SwitchFineAlignment()
        {
            this.FineAlignmentVisibility = this.NeedFineAlignment ? Visibility.Visible : Visibility.Collapsed;
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
                this.ICPVisibility = Visibility.Visible;
                this.NDTVisibility = Visibility.Collapsed;
            }
            else if (this.SelectedFineAlignmentMode == FineAlignmentMode.NDT)
            {
                this.NDTVisibility = Visibility.Visible;
                this.ICPVisibility = Visibility.Collapsed;
            }
            else
            {
                this.ICPVisibility = Visibility.Collapsed;
                this.NDTVisibility = Visibility.Collapsed;
            }
        }
        #endregion

        #region 提交 —— async void Submit()
        /// <summary>
        /// 提交
        /// </summary>
        public async void Submit()
        {
            #region # 验证

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
