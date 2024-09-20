using Caliburn.Micro;
using HelixToolkit.Wpf.SharpDX;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using Microsoft.Win32;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Features;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using SD.IOC.Core.Mediators;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using PerspectiveCamera = HelixToolkit.Wpf.SharpDX.PerspectiveCamera;

namespace PCLSharp.Client.ViewModels.RegistrationContext
{
    /// <summary>
    /// 点云配准首页视图模型
    /// </summary>
    public class IndexViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云通用操作接口;
        /// </summary>
        private readonly ICloudCommon _cloudCommon;

        /// <summary>
        /// 点云文件接口
        /// </summary>
        private readonly ICloudFiles _cloudFiles;

        /// <summary>
        /// 点云滤波接口
        /// </summary>
        private readonly ICloudFilters _cloudFilters;

        /// <summary>
        /// 点云关键点接口
        /// </summary>
        private readonly ICloudKeyPoints _cloudKeyPoints;

        /// <summary>
        /// 点云特征接口
        /// </summary>
        private readonly ICloudFeatures _cloudFeatures;

        /// <summary>
        /// 点云分割接口
        /// </summary>
        private readonly ICloudSegmentations _cloudSegmentations;

        /// <summary>
        /// 点云配准接口
        /// </summary>
        private readonly ICloudRegistrations _cloudRegistrations;

        /// <summary>
        /// 窗体管理器
        /// </summary>
        private readonly IWindowManager _windowManager;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public IndexViewModel(ICloudCommon cloudCommon, ICloudFiles cloudFiles, ICloudFilters cloudFilters, ICloudKeyPoints cloudKeyPoints, ICloudFeatures cloudFeatures, ICloudSegmentations cloudSegmentations, ICloudRegistrations cloudRegistrations, IWindowManager windowManager)
        {
            this._cloudCommon = cloudCommon;
            this._cloudFiles = cloudFiles;
            this._cloudFilters = cloudFilters;
            this._cloudKeyPoints = cloudKeyPoints;
            this._cloudFeatures = cloudFeatures;
            this._cloudSegmentations = cloudSegmentations;
            this._cloudRegistrations = cloudRegistrations;
            this._windowManager = windowManager;
        }

        #endregion

        #region # 属性

        #region MSAA等级 —— MSAALevel MSAALevel
        /// <summary>
        /// MSAA等级
        /// </summary>
        [DependencyProperty]
        public MSAALevel MSAALevel { get; set; }
        #endregion

        #region 是否高画质 —— bool HighImageQuality
        /// <summary>
        /// 是否高画质
        /// </summary>
        [DependencyProperty]
        public bool HighImageQuality { get; set; }
        #endregion

        #region 源文件路径 —— string SourceFilePath
        /// <summary>
        /// 源文件路径
        /// </summary>
        public string SourceFilePath { get; set; }
        #endregion

        #region 源文件格式 —— string SourceFileExtension
        /// <summary>
        /// 源文件格式
        /// </summary>
        public string SourceFileExtension { get; set; }
        #endregion

        #region 目标文件路径 —— string TargetFilePath
        /// <summary>
        /// 目标文件路径
        /// </summary>
        public string TargetFilePath { get; set; }
        #endregion

        #region 目标文件格式 —— string TargetFileExtension
        /// <summary>
        /// 目标文件格式
        /// </summary>
        public string TargetFileExtension { get; set; }
        #endregion

        #region 参考点云 —— PointGeometry3D SourceCloud
        /// <summary>
        /// 参考点云
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D SourceCloud { get; set; }
        #endregion

        #region 目标点云 —— PointGeometry3D TargetCloud
        /// <summary>
        /// 目标点云
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D TargetCloud { get; set; }
        #endregion

        #region 参考点云质心 —— PointGeometry3D SourceCentroid
        /// <summary>
        /// 参考点云质心
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D SourceCentroid { get; set; }
        #endregion

        #region 一次采样耗时 —— string SampleIDuration
        /// <summary>
        /// 一次采样耗时
        /// </summary>
        [DependencyProperty]
        public string SampleIDuration { get; set; }
        #endregion

        #region 分割耗时 —— string SegmentDuration
        /// <summary>
        /// 分割耗时
        /// </summary>
        [DependencyProperty]
        public string SegmentDuration { get; set; }
        #endregion

        #region 离群点耗时 —— string OuterRemovalDuration
        /// <summary>
        /// 离群点耗时
        /// </summary>
        [DependencyProperty]
        public string OuterRemovalDuration { get; set; }
        #endregion

        #region 关键点耗时 —— string KeyPointDuration
        /// <summary>
        /// 关键点耗时
        /// </summary>
        [DependencyProperty]
        public string KeyPointDuration { get; set; }
        #endregion

        #region 特征耗时 —— string FeatureDuration
        /// <summary>
        /// 特征耗时
        /// </summary>
        [DependencyProperty]
        public string FeatureDuration { get; set; }
        #endregion

        #region 粗配准耗时 —— string CoarseAlignmentDuration
        /// <summary>
        /// 粗配准耗时
        /// </summary>
        [DependencyProperty]
        public string CoarseAlignmentDuration { get; set; }
        #endregion

        #region 二次采样耗时 —— string SampleIIDuration
        /// <summary>
        /// 二次采样耗时
        /// </summary>
        [DependencyProperty]
        public string SampleIIDuration { get; set; }
        #endregion

        #region 精配准耗时 —— string FineAlignmentDuration
        /// <summary>
        /// 精配准耗时
        /// </summary>
        [DependencyProperty]
        public string FineAlignmentDuration { get; set; }
        #endregion

        #region 合计耗时 —— string TotalDuration
        /// <summary>
        /// 合计耗时
        /// </summary>
        [DependencyProperty]
        public string TotalDuration { get; set; }
        #endregion

        #region 参考点云采样数I —— int? SourceSampledCountI
        /// <summary>
        /// 参考点云采样数I
        /// </summary>
        [DependencyProperty]
        public int? SourceSampledCountI { get; set; }
        #endregion

        #region 目标点云采样数I —— int? TargetSampledCountI
        /// <summary>
        /// 目标点云采样数I
        /// </summary>
        [DependencyProperty]
        public int? TargetSampledCountI { get; set; }
        #endregion

        #region 参考点云采样数II —— int? SourceSampledCountII
        /// <summary>
        /// 参考点云采样数I
        /// </summary>
        [DependencyProperty]
        public int? SourceSampledCountII { get; set; }
        #endregion

        #region 目标点云采样数II —— int? TargetSampledCountII
        /// <summary>
        /// 目标点云采样数II
        /// </summary>
        [DependencyProperty]
        public int? TargetSampledCountII { get; set; }
        #endregion

        #region 参考点云分割数 —— int? SourceSegmentedCount
        /// <summary>
        /// 参考点云分割数
        /// </summary>
        [DependencyProperty]
        public int? SourceSegmentedCount { get; set; }
        #endregion

        #region 目标点云分割数 —— int? TargetSegmentedCount
        /// <summary>
        /// 目标点云分割数
        /// </summary>
        [DependencyProperty]
        public int? TargetSegmentedCount { get; set; }
        #endregion

        #region 参考点云离群数 —— int? SourceOutlierCount
        /// <summary>
        /// 参考点云离群数
        /// </summary>
        [DependencyProperty]
        public int? SourceOutlierCount { get; set; }
        #endregion

        #region 目标点云离群数 —— int? TargetOutlierCount
        /// <summary>
        /// 目标点云离群数
        /// </summary>
        [DependencyProperty]
        public int? TargetOutlierCount { get; set; }
        #endregion

        #region 参考点云关键点数 —— int? SourceKeyPointsCount
        /// <summary>
        /// 参考点云关键点数
        /// </summary>
        [DependencyProperty]
        public int? SourceKeyPointsCount { get; set; }
        #endregion

        #region 目标点云关键点数 —— int? TargetKeyPointsCount
        /// <summary>
        /// 目标点云关键点数
        /// </summary>
        [DependencyProperty]
        public int? TargetKeyPointsCount { get; set; }
        #endregion

        #region 粗配准是否收敛 —— bool? CoarseHasConverged
        /// <summary>
        /// 粗配准是否收敛
        /// </summary>
        [DependencyProperty]
        public bool? CoarseHasConverged { get; set; }
        #endregion

        #region 粗配准拟合分数 —— float? CoarseFitnessScore
        /// <summary>
        /// 粗配准拟合分数
        /// </summary>
        [DependencyProperty]
        public float? CoarseFitnessScore { get; set; }
        #endregion

        #region 粗配准RT矩阵 —— string CoarseMatrix
        /// <summary>
        /// 粗配准RT矩阵
        /// </summary>
        [DependencyProperty]
        public string CoarseMatrix { get; set; }
        #endregion

        #region 精配准是否收敛 —— bool? FineHasConverged
        /// <summary>
        /// 精配准是否收敛
        /// </summary>
        [DependencyProperty]
        public bool? FineHasConverged { get; set; }
        #endregion

        #region 精配准拟合分数 —— float? FineFitnessScore
        /// <summary>
        /// 精配准拟合分数
        /// </summary>
        [DependencyProperty]
        public float? FineFitnessScore { get; set; }
        #endregion

        #region 精配准RT矩阵 —— string FineMatrix
        /// <summary>
        /// 精配准RT矩阵
        /// </summary>
        [DependencyProperty]
        public string FineMatrix { get; set; }
        #endregion

        #region 最终RT矩阵 —— string FinalMatrix
        /// <summary>
        /// 最终RT矩阵
        /// </summary>
        [DependencyProperty]
        public string FinalMatrix { get; set; }
        #endregion

        #region 相机 —— PerspectiveCamera Camera
        /// <summary>
        /// 相机
        /// </summary>
        [DependencyProperty]
        public PerspectiveCamera Camera { get; set; }
        #endregion

        #region 配准参数视图模型 —— ParamViewModel ParamViewModel
        /// <summary>
        /// 配准参数视图模型
        /// </summary>
        [DependencyProperty]
        public ParamViewModel ParamViewModel { get; set; }
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
            //默认值
            this.MSAALevel = MSAALevel.Maximum;
            this.HighImageQuality = true;

            //初始化相机
            this.ResetCamera();

            return base.OnInitializeAsync(cancellationToken);
        }
        #endregion


        //Actions

        #region 打开参考点云 —— async void OpenSourceCloud()
        /// <summary>
        /// 打开参考点云
        /// </summary>
        public async void OpenSourceCloud()
        {
            OpenFileDialog openFileDialog = new OpenFileDialog
            {
                Filter = Constants.OpenCloudExtFilter,
                AddExtension = true,
                RestoreDirectory = true
            };
            if (openFileDialog.ShowDialog() == true)
            {
                this.Busy();

                this.SourceFilePath = openFileDialog.FileName;
                this.SourceFileExtension = Path.GetExtension(this.SourceFilePath);
                await this.ReloadSourceCloud();

                this.Idle();
            }
        }
        #endregion

        #region 打开目标点云 —— async void OpenTargetCloud()
        /// <summary>
        /// 打开目标点云
        /// </summary>
        public async void OpenTargetCloud()
        {
            OpenFileDialog openFileDialog = new OpenFileDialog
            {
                Filter = Constants.OpenCloudExtFilter,
                AddExtension = true,
                RestoreDirectory = true
            };
            if (openFileDialog.ShowDialog() == true)
            {
                this.Busy();

                this.TargetFilePath = openFileDialog.FileName;
                this.TargetFileExtension = Path.GetExtension(this.TargetFilePath);
                await this.ReloadTargetCloud();

                this.Idle();
            }
        }
        #endregion

        #region 设置参数 —— async void SetParameters()
        /// <summary>
        /// 设置参数
        /// </summary>
        public async void SetParameters()
        {
            #region # 验证

            if (this.SourceCloud == null)
            {
                MessageBox.Show("参考点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.TargetCloud == null)
            {
                MessageBox.Show("目标点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            ParamViewModel paramViewModel = this.ParamViewModel ?? ResolveMediator.Resolve<ParamViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(paramViewModel);
            if (result == true)
            {
                this.ParamViewModel = paramViewModel;
            }
        }
        #endregion

        #region 执行配准 —— async void ExecuteAlignment()
        /// <summary>
        /// 执行配准
        /// </summary>
        public async void ExecuteAlignment()
        {
            //清空结果
            this.SampleIDuration = null;
            this.SampleIIDuration = null;
            this.SegmentDuration = null;
            this.OuterRemovalDuration = null;
            this.KeyPointDuration = null;
            this.FeatureDuration = null;
            this.CoarseAlignmentDuration = null;
            this.FineAlignmentDuration = null;
            this.TotalDuration = null;
            this.SourceSampledCountI = null;
            this.TargetSampledCountI = null;
            this.SourceSampledCountII = null;
            this.TargetSampledCountII = null;
            this.SourceSegmentedCount = null;
            this.TargetSegmentedCount = null;
            this.SourceOutlierCount = null;
            this.TargetOutlierCount = null;
            this.SourceKeyPointsCount = null;
            this.TargetKeyPointsCount = null;
            this.CoarseHasConverged = null;
            this.CoarseFitnessScore = null;
            this.CoarseMatrix = null;
            this.FineHasConverged = null;
            this.FineFitnessScore = null;
            this.FineMatrix = null;
            this.FinalMatrix = null;

            #region # 验证

            if (this.SourceCloud == null)
            {
                MessageBox.Show("参考点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.TargetCloud == null)
            {
                MessageBox.Show("目标点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.ParamViewModel == null)
            {
                MessageBox.Show("配准参数未设置！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            MessageBoxResult confirmed = MessageBox.Show("确定要执行配准？", "警告", MessageBoxButton.YesNo, MessageBoxImage.Question);
            if (confirmed != MessageBoxResult.Yes)
            {
                return;
            }

            #endregion

            Stopwatch sampleIWatch = new Stopwatch();
            Stopwatch sampleIIWatch = new Stopwatch();
            Stopwatch segmentWatch = new Stopwatch();
            Stopwatch outlierRemovalWatch = new Stopwatch();
            Stopwatch keyPointWatch = new Stopwatch();
            Stopwatch featureWatch = new Stopwatch();
            Stopwatch coarseWatch = new Stopwatch();
            Stopwatch fineWatch = new Stopwatch();
            Stopwatch totalWatch = new Stopwatch();

            this.Busy();
            totalWatch.Start();

            //提取原始点云
            Point3F[] originalSourcePoints = this.SourceCloud.Points.ToPoint3Fs().ToArray();
            Point3F[] originalTargetPoints = this.TargetCloud.Points.ToPoint3Fs().ToArray();
            Point3F[] sourceBufferPoints = new Point3F[originalSourcePoints.Length];
            Point3F[] targetBufferPoints = new Point3F[originalTargetPoints.Length];
            Array.Copy(originalSourcePoints, sourceBufferPoints, originalSourcePoints.Length);
            Array.Copy(originalTargetPoints, targetBufferPoints, originalTargetPoints.Length);

            //一次采样
            if (this.ParamViewModel.SampleIEnabled)
            {
                sampleIWatch.Start();
                sourceBufferPoints = await Task.Run(() => this._cloudFilters.ApplyVoxelGrid(sourceBufferPoints, this.ParamViewModel.SampleILeafSize!.Value));
                targetBufferPoints = await Task.Run(() => this._cloudFilters.ApplyVoxelGrid(targetBufferPoints, this.ParamViewModel.SampleILeafSize!.Value));
                sampleIWatch.Stop();
                this.SampleIDuration = sampleIWatch.Elapsed.ToString(Constants.DurationFormat);
                this.SourceSampledCountI = sourceBufferPoints.Length;
                this.TargetSampledCountI = targetBufferPoints.Length;
            }

            //粗配准
            AlignmentResult? coarseAlignmentResult = null;
            if (this.ParamViewModel.CoarseAlignmentEnabled)
            {
                //分割最大簇
                if (this.ParamViewModel.SegmentEnabled)
                {
                    segmentWatch.Start();
                    Point3F[][] sourceClusters = await Task.Run(() => this._cloudSegmentations.EuclidClusterSegment(sourceBufferPoints, this.ParamViewModel.ClusterTolerance!.Value, this.ParamViewModel.MinClusterSize!.Value, this.ParamViewModel.MaxClusterSize!.Value));
                    Point3F[][] targetClusters = await Task.Run(() => this._cloudSegmentations.EuclidClusterSegment(targetBufferPoints, this.ParamViewModel.ClusterTolerance!.Value, this.ParamViewModel.MinClusterSize!.Value, this.ParamViewModel.MaxClusterSize!.Value));
                    if (sourceClusters.Any())
                    {
                        sourceBufferPoints = sourceClusters[0];
                    }
                    if (targetClusters.Any())
                    {
                        targetBufferPoints = targetClusters[0];
                    }
                    segmentWatch.Stop();
                    this.SegmentDuration = segmentWatch.Elapsed.ToString(Constants.DurationFormat);
                    this.SourceSegmentedCount = sourceBufferPoints.Length;
                    this.TargetSegmentedCount = targetBufferPoints.Length;
                }

                //离群点移除
                if (this.ParamViewModel.OutlierRemovalEnabled)
                {
                    outlierRemovalWatch.Start();
                    sourceBufferPoints = await Task.Run(() => this._cloudFilters.ApplyStatisticalOutlierRemoval(sourceBufferPoints, this.ParamViewModel.MeanK!.Value, this.ParamViewModel.StddevMult!.Value));
                    targetBufferPoints = await Task.Run(() => this._cloudFilters.ApplyStatisticalOutlierRemoval(targetBufferPoints, this.ParamViewModel.MeanK!.Value, this.ParamViewModel.StddevMult!.Value));
                    outlierRemovalWatch.Stop();
                    this.OuterRemovalDuration = outlierRemovalWatch.Elapsed.ToString(Constants.DurationFormat);
                    this.SourceOutlierCount = sourceBufferPoints.Length;
                    this.TargetOutlierCount = targetBufferPoints.Length;
                }

                //K-FPCS
                if (this.ParamViewModel.SelectedCoarseAlignmentMode == CoarseAlignmentMode.KFPCS)
                {
                    //粗配准
                    coarseWatch.Start();
                    coarseAlignmentResult = this._cloudRegistrations.AlignKFPCS(sourceBufferPoints, targetBufferPoints, this.ParamViewModel.ApproxOverlap!.Value, this.ParamViewModel.Lambda!.Value, this.ParamViewModel.Delta!.Value, this.ParamViewModel.NeedToNormalize!.Value, this.ParamViewModel.SamplesCount!.Value, this.ParamViewModel.MaxComputationTime!.Value, this.ParamViewModel.ThreadsCount!.Value);
                    sourceBufferPoints = await Task.Run(() => this._cloudCommon.MatrixTransform(sourceBufferPoints, coarseAlignmentResult.Value.Matrix));
                    coarseWatch.Stop();
                    this.CoarseAlignmentDuration = coarseWatch.Elapsed.ToString(Constants.DurationFormat);
                }
                //SAC-IA
                else if (this.ParamViewModel.SelectedCoarseAlignmentMode == CoarseAlignmentMode.SACIA)
                {
                    //ISS关键点
                    keyPointWatch.Start();
                    Point3F[] sourceKeyPoints = await Task.Run(() => this._cloudKeyPoints.DetectISS(sourceBufferPoints, this.ParamViewModel.SalientRadius!.Value, this.ParamViewModel.NonMaxRadius!.Value, this.ParamViewModel.Threshold21!.Value, this.ParamViewModel.Threshold32!.Value, this.ParamViewModel.MinNeighborsCount!.Value, this.ParamViewModel.ThreadsCount!.Value));
                    Point3F[] targetKeyPoints = await Task.Run(() => this._cloudKeyPoints.DetectISS(targetBufferPoints, this.ParamViewModel.SalientRadius!.Value, this.ParamViewModel.NonMaxRadius!.Value, this.ParamViewModel.Threshold21!.Value, this.ParamViewModel.Threshold32!.Value, this.ParamViewModel.MinNeighborsCount!.Value, this.ParamViewModel.ThreadsCount!.Value));
                    keyPointWatch.Stop();
                    this.KeyPointDuration = keyPointWatch.Elapsed.ToString(Constants.DurationFormat);
                    this.SourceKeyPointsCount = sourceKeyPoints.Length;
                    this.TargetKeyPointsCount = targetKeyPoints.Length;

                    //FPFH特征
                    featureWatch.Start();
                    FPFHSignature33F[] sourceDescriptors = await Task.Run(() => this._cloudFeatures.ComputeFPFH(sourceKeyPoints, this.ParamViewModel.NormalK!.Value, this.ParamViewModel.FeatureK!.Value, this.ParamViewModel.ThreadsCount!.Value));
                    FPFHSignature33F[] targetDescriptors = await Task.Run(() => this._cloudFeatures.ComputeFPFH(targetKeyPoints, this.ParamViewModel.NormalK!.Value, this.ParamViewModel.FeatureK!.Value, this.ParamViewModel.ThreadsCount!.Value));
                    featureWatch.Stop();
                    this.FeatureDuration = featureWatch.Elapsed.ToString(Constants.DurationFormat);

                    //粗配准
                    coarseWatch.Start();
                    coarseAlignmentResult = this._cloudRegistrations.AlignSACIA(sourceKeyPoints, sourceDescriptors, targetKeyPoints, targetDescriptors, this.ParamViewModel.MinSampleDistance!.Value, this.ParamViewModel.SamplesCount!.Value, this.ParamViewModel.CorrespondenceRandomness!.Value);
                    sourceBufferPoints = await Task.Run(() => this._cloudCommon.MatrixTransform(sourceBufferPoints, coarseAlignmentResult.Value.Matrix));
                    coarseWatch.Stop();
                    this.CoarseAlignmentDuration = coarseWatch.Elapsed.ToString(Constants.DurationFormat);
                }
                else
                {
                    throw new NotSupportedException();
                }
            }

            //二次采样
            if (this.ParamViewModel.SampleIIEnabled)
            {
                sampleIIWatch.Start();
                sourceBufferPoints = await Task.Run(() => this._cloudFilters.ApplyVoxelGrid(sourceBufferPoints, this.ParamViewModel.SampleIILeafSize!.Value));
                targetBufferPoints = await Task.Run(() => this._cloudFilters.ApplyVoxelGrid(targetBufferPoints, this.ParamViewModel.SampleIILeafSize!.Value));
                sampleIIWatch.Stop();
                this.SampleIIDuration = sampleIIWatch.Elapsed.ToString(Constants.DurationFormat);
                this.SourceSampledCountII = sourceBufferPoints.Length;
                this.TargetSampledCountII = targetBufferPoints.Length;
            }

            //精配准
            AlignmentResult? fineAlignmentResult = null;
            if (this.ParamViewModel.FineAlignmentEnabled)
            {
                fineWatch.Start();
                //ICP: Point to point
                if (this.ParamViewModel.SelectedFineAlignmentMode == FineAlignmentMode.PointToPoint)
                {
                    fineAlignmentResult = this._cloudRegistrations.AlignPointToPoint(sourceBufferPoints, targetBufferPoints, this.ParamViewModel.MaxCorrespondenceDistance!.Value, this.ParamViewModel.TransformationEpsilon!.Value,
                        this.ParamViewModel.EuclideanFitnessEpsilon!.Value, this.ParamViewModel.MaximumIterations!.Value);
                }
                //ICP: Point to plane
                else if (this.ParamViewModel.SelectedFineAlignmentMode == FineAlignmentMode.PointToPlane)
                {
                    fineAlignmentResult = this._cloudRegistrations.AlignPointToPlane(sourceBufferPoints, targetBufferPoints, this.ParamViewModel.NormalK!.Value, this.ParamViewModel.MaxCorrespondenceDistance!.Value, this.ParamViewModel.TransformationEpsilon!.Value, this.ParamViewModel.EuclideanFitnessEpsilon!.Value, this.ParamViewModel.MaximumIterations!.Value, this.ParamViewModel.ThreadsCount!.Value);
                }
                //GICP
                else if (this.ParamViewModel.SelectedFineAlignmentMode == FineAlignmentMode.GICP)
                {
                    fineAlignmentResult = this._cloudRegistrations.AlignGICP(sourceBufferPoints, targetBufferPoints, this.ParamViewModel.MaxCorrespondenceDistance!.Value, this.ParamViewModel.TransformationEpsilon!.Value, this.ParamViewModel.EuclideanFitnessEpsilon!.Value, this.ParamViewModel.MaximumIterations!.Value);
                }
                //NDT
                else if (this.ParamViewModel.SelectedFineAlignmentMode == FineAlignmentMode.NDT)
                {
                    fineAlignmentResult = this._cloudRegistrations.AlignNDT(sourceBufferPoints, targetBufferPoints, this.ParamViewModel.Resolution!.Value, this.ParamViewModel.StepSize!.Value, this.ParamViewModel.TransformationEpsilon!.Value, this.ParamViewModel.MaximumIterations!.Value);
                }
                else
                {
                    throw new NotSupportedException();
                }
                fineWatch.Stop();
                this.FineAlignmentDuration = fineWatch.Elapsed.ToString(Constants.DurationFormat);
            }

            //解析配准结果
            const int rowsCount = 4;
            const int colsCount = 4;
            Matrix<float> coarseMatrix = DenseMatrix.CreateIdentity(rowsCount);
            Matrix<float> fineMatrix = DenseMatrix.CreateIdentity(rowsCount);
            for (int rowIndex = 0; rowIndex < rowsCount; rowIndex++)
            {
                for (int colIndex = 0; colIndex < colsCount; colIndex++)
                {
                    int index = rowIndex * colsCount + colIndex;
                    if (coarseAlignmentResult.HasValue)
                    {
                        coarseMatrix[rowIndex, colIndex] = coarseAlignmentResult!.Value.Matrix[index];
                    }
                    if (fineAlignmentResult.HasValue)
                    {
                        fineMatrix[rowIndex, colIndex] = fineAlignmentResult!.Value.Matrix[index];
                    }
                }
            }
            Matrix<float> finalMatrix = fineMatrix * coarseMatrix;
            this.CoarseHasConverged = coarseAlignmentResult?.HasConverged;
            this.CoarseFitnessScore = coarseAlignmentResult?.FitnessScore;
            this.CoarseMatrix = coarseMatrix.ToMatrixString(Constants.MatrixFormat);
            this.FineHasConverged = fineAlignmentResult?.HasConverged;
            this.FineFitnessScore = fineAlignmentResult?.FitnessScore;
            this.FineMatrix = fineMatrix.ToMatrixString(Constants.MatrixFormat);
            this.FinalMatrix = finalMatrix.ToMatrixString(Constants.MatrixFormat);

            totalWatch.Stop();
            this.TotalDuration = totalWatch.Elapsed.ToString(Constants.DurationFormat);

            //原始点云转换
            if (coarseAlignmentResult.HasValue)
            {
                originalSourcePoints = await Task.Run(() => this._cloudCommon.MatrixTransform(originalSourcePoints, coarseAlignmentResult.Value.Matrix));
            }
            if (fineAlignmentResult.HasValue)
            {
                originalSourcePoints = await Task.Run(() => this._cloudCommon.MatrixTransform(originalSourcePoints, fineAlignmentResult.Value.Matrix));
            }
            this.SourceCloud = originalSourcePoints.ToPointGeometry3D();

            this.Idle();
        }
        #endregion

        #region 切换画质 —— void SwitchImageQuality()
        /// <summary>
        /// 切换画质
        /// </summary>
        public void SwitchImageQuality()
        {
            this.MSAALevel = this.HighImageQuality ? MSAALevel.Maximum : MSAALevel.Disable;
        }
        #endregion

        #region 指向质心 —— async void LookAtCentroid()
        /// <summary>
        /// 指向质心
        /// </summary>
        public async void LookAtCentroid()
        {
            #region # 验证

            if (this.SourceCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            Point3F centroid;
            if (this.SourceCentroid != null)
            {
                Vector3 position = this.SourceCentroid.Positions.Single();
                centroid = position.ToPoint3F();
            }
            else
            {
                IEnumerable<Point3F> points = this.SourceCloud.Points.ToPoint3Fs();
                centroid = await Task.Run(() => this._cloudCommon.EstimateCentroid(points));
                this.SourceCentroid = new[] { centroid }.ToPointGeometry3D();
            }

            this.Camera.LookAt(centroid.ToPoint(), 200);

            this.Idle();
        }
        #endregion

        #region 重置点云 —— async void ResetPointCloud()
        /// <summary>
        /// 重置点云
        /// </summary>
        public async void ResetPointCloud()
        {
            this.Busy();

            if (!string.IsNullOrWhiteSpace(this.SourceFilePath))
            {
                await this.ReloadSourceCloud();
            }
            if (!string.IsNullOrWhiteSpace(this.TargetFilePath))
            {
                await this.ReloadTargetCloud();
            }

            this.Idle();
        }
        #endregion

        #region 重置相机 —— void ResetCamera()
        /// <summary>
        /// 重置相机
        /// </summary>
        public void ResetCamera()
        {
            this.Camera = new PerspectiveCamera
            {
                LookDirection = new Vector3D(0, 0, -200),
                UpDirection = new Vector3D(0, 1, 0),
                Position = new Point3D(0, 0, 200),
                NearPlaneDistance = 0.125,
                FarPlaneDistance = double.PositiveInfinity,
                FieldOfView = 30
            };
        }
        #endregion


        //Events

        #region 键盘按下事件 —— void OnKeyDown()
        /// <summary>
        /// 键盘按下事件
        /// </summary>
        public void OnKeyDown()
        {
            if (Keyboard.IsKeyDown(Key.F5))
            {
                this.ResetPointCloud();
            }
        }
        #endregion


        //Private

        #region 加载参考点云 —— async Task ReloadSourceCloud()
        /// <summary>
        /// 加载参考点云
        /// </summary>
        private async Task ReloadSourceCloud()
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(this.SourceFilePath))
            {
                return;
            }
            if (string.IsNullOrWhiteSpace(this.SourceFileExtension))
            {
                return;
            }

            #endregion

            Point3F[] points = this.SourceFileExtension switch
            {
                Constants.PCD => await Task.Run(() => this._cloudFiles.LoadPCD(this.SourceFilePath)),
                Constants.PLY => await Task.Run(() => this._cloudFiles.LoadPLY(this.SourceFilePath)),
                Constants.OBJ => await Task.Run(() => this._cloudFiles.LoadOBJ(this.SourceFilePath)),
                _ => throw new NotSupportedException("不支持的点云格式！")
            };

            //过滤NaN
            points = points.FilterNaN();

            this.SourceCloud = points.ToPointGeometry3D();
        }
        #endregion

        #region 加载目标点云 —— async Task ReloadTargetCloud()
        /// <summary>
        /// 加载目标点云
        /// </summary>
        private async Task ReloadTargetCloud()
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(this.TargetFilePath))
            {
                return;
            }
            if (string.IsNullOrWhiteSpace(this.TargetFileExtension))
            {
                return;
            }

            #endregion

            Point3F[] points = this.TargetFileExtension switch
            {
                Constants.PCD => await Task.Run(() => this._cloudFiles.LoadPCD(this.TargetFilePath)),
                Constants.PLY => await Task.Run(() => this._cloudFiles.LoadPLY(this.TargetFilePath)),
                Constants.OBJ => await Task.Run(() => this._cloudFiles.LoadOBJ(this.TargetFilePath)),
                _ => throw new NotSupportedException("不支持的点云格式！")
            };

            //过滤NaN
            points = points.FilterNaN();

            this.TargetCloud = points.ToPointGeometry3D();
        }
        #endregion

        #endregion
    }
}
