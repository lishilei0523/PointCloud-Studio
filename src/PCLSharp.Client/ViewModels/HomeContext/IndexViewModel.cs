using Caliburn.Micro;
using HelixToolkit.Wpf.SharpDX;
using Microsoft.Win32;
using PCLSharp.Client.ViewModels.CommonContext;
using PCLSharp.Client.ViewModels.FilterContext;
using PCLSharp.Client.ViewModels.KeyPointContext;
using PCLSharp.Client.ViewModels.NormalContext;
using PCLSharp.HelixDX.WPF;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Enums;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using SD.Infrastructure.WPF.Extensions;
using SD.IOC.Core.Mediators;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using Color = System.Windows.Media.Color;
using PerspectiveCamera = HelixToolkit.Wpf.SharpDX.PerspectiveCamera;

namespace PCLSharp.Client.ViewModels.HomeContext
{
    /// <summary>
    /// 首页视图模型
    /// </summary>
    public class IndexViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云文件接口
        /// </summary>
        private readonly ICloudFiles _cloudFiles;

        /// <summary>
        /// 点云滤波接口
        /// </summary>
        private readonly ICloudFilters _cloudFilters;

        /// <summary>
        /// 点云法向量接口
        /// </summary>
        private readonly ICloudNormals _cloudNormals;

        /// <summary>
        /// 点云关键点接口
        /// </summary>
        private readonly ICloudKeyPoints _cloudKeyPoints;

        /// <summary>
        /// 窗体管理器
        /// </summary>
        private readonly IWindowManager _windowManager;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public IndexViewModel(ICloudFiles cloudFiles, ICloudFilters cloudFilters, ICloudNormals cloudNormals, ICloudKeyPoints cloudKeyPoints, IWindowManager windowManager)
        {
            this._cloudFiles = cloudFiles;
            this._cloudFilters = cloudFilters;
            this._cloudNormals = cloudNormals;
            this._cloudKeyPoints = cloudKeyPoints;
            this._windowManager = windowManager;
        }

        #endregion

        #region # 属性

        #region 文件路径 —— string FilePath
        /// <summary>
        /// 文件路径
        /// </summary>
        [DependencyProperty]
        public string FilePath { get; set; }
        #endregion

        #region 文件格式 —— string FileExtension
        /// <summary>
        /// 文件格式
        /// </summary>
        [DependencyProperty]
        public string FileExtension { get; set; }
        #endregion

        #region 点云颜色 —— Color PointColor
        /// <summary>
        /// 点云颜色
        /// </summary>
        [DependencyProperty]
        public Color PointColor { get; set; }
        #endregion

        #region 标签颜色 —— Color LabelColor
        /// <summary>
        /// 标签颜色
        /// </summary>
        [DependencyProperty]
        public Color LabelColor { get; set; }
        #endregion

        #region 背景颜色 —— Color BackgroundColor
        /// <summary>
        /// 背景颜色
        /// </summary>
        [DependencyProperty]
        public Color BackgroundColor { get; set; }
        #endregion

        #region 关键点颜色 —— Color KeyPointColor
        /// <summary>
        /// 关键点颜色
        /// </summary>
        [DependencyProperty]
        public Color KeyPointColor { get; set; }
        #endregion

        #region 点云颜色类型 —— PointColorType? CloudColorType
        /// <summary>
        /// 点云颜色类型
        /// </summary>
        public PointColorType? CloudColorType { get; set; }
        #endregion

        #region 原始点云 —— PointGeometry3D OriginalPointCloud
        /// <summary>
        /// 原始点云
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D OriginalPointCloud { get; set; }
        #endregion

        #region 效果点云 —— PointGeometry3D EffectivePointCloud
        /// <summary>
        /// 效果点云
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D EffectivePointCloud { get; set; }
        #endregion

        #region 效果点云质心 —— PointGeometry3D EffectiveCentroid
        /// <summary>
        /// 效果点云质心
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D EffectiveCentroid { get; set; }
        #endregion

        #region 效果点云关键点 —— PointGeometry3D EffectiveKeyPoints
        /// <summary>
        /// 效果点云关键点
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D EffectiveKeyPoints { get; set; }
        #endregion

        #region 效果点云法向量列表 —— ObservableCollection<LineGeometryModel3D> EffectiveNormals
        /// <summary>
        /// 效果点云法向量列表
        /// </summary>
        [DependencyProperty]
        public ObservableCollection<LineGeometryModel3D> EffectiveNormals { get; set; }
        #endregion

        #region 相机 —— PerspectiveCamera Camera
        /// <summary>
        /// 相机
        /// </summary>
        [DependencyProperty]
        public PerspectiveCamera Camera { get; set; }
        #endregion

        #endregion

        #region # 方法

        //Initializations

        #region 初始化 —— Task OnInitializeAsync(CancellationToken cancellationToken)
        /// <summary>
        /// 初始化
        /// </summary>
        protected override Task OnInitializeAsync(CancellationToken cancellationToken)
        {
            //默认值
            this.LabelColor = Colors.Black;
            this.BackgroundColor = Colors.LightGray;
            this.EffectiveNormals = new ObservableCollection<LineGeometryModel3D>();

            //初始化相机
            this.ResetCamera();

            return base.OnInitializeAsync(cancellationToken);
        }
        #endregion


        //文件

        #region 打开灰度点云 —— async void OpenMonoCloud()
        /// <summary>
        /// 打开灰度点云
        /// </summary>
        public async void OpenMonoCloud()
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

                this.FilePath = openFileDialog.FileName;
                this.FileExtension = Path.GetExtension(this.FilePath);
                this.CloudColorType = PointColorType.Mono;
                await this.ReloadCloud();

                this.Idle();
            }
        }
        #endregion

        #region 打开彩色点云 —— async void OpenColorCloud()
        /// <summary>
        /// 打开彩色点云
        /// </summary>
        public async void OpenColorCloud()
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

                this.FilePath = openFileDialog.FileName;
                this.FileExtension = Path.GetExtension(this.FilePath);
                this.CloudColorType = PointColorType.RGBA;
                await this.ReloadCloud();

                this.Idle();
            }
        }
        #endregion

        #region 关闭点云 —— void CloseCloud()
        /// <summary>
        /// 关闭点云
        /// </summary>
        public void CloseCloud()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            MessageBoxResult result = MessageBox.Show("是否保存？", "提示", MessageBoxButton.YesNoCancel, MessageBoxImage.Question);
            if (result == MessageBoxResult.Yes)
            {
                this.SaveCloud();
            }
            if (result == MessageBoxResult.Yes || result == MessageBoxResult.No)
            {
                this.FilePath = null;
                this.FileExtension = null;
                this.OriginalPointCloud = null;
                this.EffectivePointCloud = null;
                this.CloudColorType = null;
                this.EffectiveNormals.Clear();
            }
        }
        #endregion

        #region 保存点云 —— async void SaveCloud()
        /// <summary>
        /// 保存点云
        /// </summary>
        public async void SaveCloud()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
            if (this.FileExtension == Constants.PCD)
            {
                await Task.Run(() => this._cloudFiles.SaveTextPCD(points, this.FilePath));
            }
            if (this.FileExtension == Constants.PLY)
            {
                await Task.Run(() => this._cloudFiles.SaveTextPLY(points, this.FilePath));
            }
            if (this.FileExtension == Constants.OBJ)
            {
                this.FilePath = this.FilePath.Replace(Constants.OBJ, Constants.PCD);
                await Task.Run(() => this._cloudFiles.SaveTextPCD(points, this.FilePath));
            }

            await this.ReloadCloud();

            this.Idle();
            this.ToastSuccess("保存成功！");
        }
        #endregion

        #region 另存为点云 —— async void SaveAsCloud()
        /// <summary>
        /// 另存为点云
        /// </summary>
        public async void SaveAsCloud()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            SaveFileDialog saveFileDialog = new SaveFileDialog
            {
                Filter = Constants.SaveCloudExtFilter,
                FileName = $"{Path.GetFileNameWithoutExtension(this.FilePath)} - 副本",
                AddExtension = true,
                RestoreDirectory = true
            };
            if (saveFileDialog.ShowDialog() == true)
            {
                this.Busy();

                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                string filePath = saveFileDialog.FileName;
                string fileExt = Path.GetExtension(filePath);
                if (fileExt == Constants.PCD)
                {
                    await Task.Run(() => this._cloudFiles.SaveTextPCD(points, filePath));
                }
                else if (fileExt == Constants.PLY)
                {
                    await Task.Run(() => this._cloudFiles.SaveTextPLY(points, filePath));
                }
                else
                {
                    throw new NotSupportedException("不支持的点云格式！");
                }

                this.Idle();
                this.ToastSuccess("保存成功！");
            }
        }
        #endregion


        //编辑

        #region 刷新点云 —— async void RefreshCloud()
        /// <summary>
        /// 刷新点云
        /// </summary>
        public async void RefreshCloud()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            await this.ReloadCloud();

            this.Idle();
        }
        #endregion

        #region 设置点云颜色 —— async void SetCloudColor()
        /// <summary>
        /// 设置点云颜色
        /// </summary>
        public async void SetCloudColor()
        {
            SelectColorViewModel viewModel = ResolveMediator.Resolve<SelectColorViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.PointColor = viewModel.Color!.Value;
            }
        }
        #endregion

        #region 设置背景颜色 —— async void SetBackgroundColor()
        /// <summary>
        /// 设置背景颜色
        /// </summary>
        public async void SetBackgroundColor()
        {
            SelectColorViewModel viewModel = ResolveMediator.Resolve<SelectColorViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.BackgroundColor = viewModel.Color!.Value;
                this.LabelColor = this.BackgroundColor.Invert();
            }
        }
        #endregion

        #region 设置关键点颜色 —— async void SetKeyPointColor()
        /// <summary>
        /// 设置关键点颜色
        /// </summary>
        public async void SetKeyPointColor()
        {
            SelectColorViewModel viewModel = ResolveMediator.Resolve<SelectColorViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.KeyPointColor = viewModel.Color!.Value;
            }
        }
        #endregion

        #region 指向质心 —— async void LookAtCentroid()
        /// <summary>
        /// 指向质心
        /// </summary>
        public async void LookAtCentroid()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            Point3F centroid;
            if (this.EffectiveCentroid != null)
            {
                Vector3 position = this.EffectiveCentroid.Positions.Single();
                centroid = position.ToPoint3F();
            }
            else
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                centroid = await Task.Run(() => this._cloudNormals.EstimateCentroid(points));
                this.EffectiveCentroid = new[] { centroid }.ToPointGeometry3D();
            }

            this.Camera.LookAt(centroid.ToPoint(), 200);

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
                LookDirection = new Vector3D(0, 0, -2),
                UpDirection = new Vector3D(0, 1, 0),
                Position = new Point3D(0, 0, 2),
                NearPlaneDistance = 0.125,
                FarPlaneDistance = double.PositiveInfinity,
                FieldOfView = 30
            };
        }
        #endregion


        //滤波

        #region 适用直通滤波 —— async void ApplyPassThrogh()
        /// <summary>
        /// 适用直通滤波
        /// </summary>
        public async void ApplyPassThrogh()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            PassThroghViewModel viewModel = ResolveMediator.Resolve<PassThroghViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyPassThrogh(points, viewModel.SelectedAxis, viewModel.LimitMin!.Value, viewModel.LimitMax!.Value));

                IEnumerable<Vector3> positions = filterdPoints.ToVector3s();
                this.EffectivePointCloud = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
            }

            this.Idle();
        }
        #endregion

        #region 适用随机采样 —— async void ApplyRandomSampling()
        /// <summary>
        /// 适用随机采样
        /// </summary>
        public async void ApplyRandomSampling()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            RandomSampleViewModel viewModel = ResolveMediator.Resolve<RandomSampleViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyRandomSampling(points, viewModel.Seed!.Value, viewModel.SamplesCount!.Value));

                IEnumerable<Vector3> positions = filterdPoints.ToVector3s();
                this.EffectivePointCloud = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
            }

            this.Idle();
        }
        #endregion

        #region 适用均匀采样 —— async void ApplyUniformSampling()
        /// <summary>
        /// 适用均匀采样
        /// </summary>
        public async void ApplyUniformSampling()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            UniformSampleViewModel viewModel = ResolveMediator.Resolve<UniformSampleViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyUniformSampling(points, viewModel.Radius!.Value));

                IEnumerable<Vector3> positions = filterdPoints.ToVector3s();
                this.EffectivePointCloud = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
            }

            this.Idle();
        }
        #endregion

        #region 适用体素降采样 —— async void ApplyVoxelGrid()
        /// <summary>
        /// 适用体素降采样
        /// </summary>
        public async void ApplyVoxelGrid()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            VoxelGridViewModel viewModel = ResolveMediator.Resolve<VoxelGridViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyVoxelGrid(points, viewModel.LeafSize!.Value));

                IEnumerable<Vector3> positions = filterdPoints.ToVector3s();
                this.EffectivePointCloud = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
            }

            this.Idle();
        }
        #endregion

        #region 适用近似体素降采样 —— async void ApplyApproximateVoxelGrid()
        /// <summary>
        /// 适用近似体素降采样
        /// </summary>
        public async void ApplyApproximateVoxelGrid()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            ApprVoxelGridViewModel viewModel = ResolveMediator.Resolve<ApprVoxelGridViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyApproximateVoxelGrid(points, viewModel.LeafSize!.Value));

                IEnumerable<Vector3> positions = filterdPoints.ToVector3s();
                this.EffectivePointCloud = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
            }

            this.Idle();
        }
        #endregion

        #region 适用统计离群点移除 —— async void ApplyStatisticalOutlierRemoval()
        /// <summary>
        /// 适用统计离群点移除
        /// </summary>
        public async void ApplyStatisticalOutlierRemoval()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            StatOutlierRemovalViewModel viewModel = ResolveMediator.Resolve<StatOutlierRemovalViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyStatisticalOutlierRemoval(points, viewModel.MeanK!.Value, viewModel.StddevMult!.Value));

                IEnumerable<Vector3> positions = filterdPoints.ToVector3s();
                this.EffectivePointCloud = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
            }

            this.Idle();
        }
        #endregion

        #region 适用半径离群点移除 —— async void ApplyRadiusOutlierRemoval()
        /// <summary>
        /// 适用半径离群点移除
        /// </summary>
        public async void ApplyRadiusOutlierRemoval()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            RadiusOutlierRemovalViewModel viewModel = ResolveMediator.Resolve<RadiusOutlierRemovalViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyRadiusOutlierRemoval(points, viewModel.Radius!.Value, viewModel.MinNeighborsInRadius!.Value));

                IEnumerable<Vector3> positions = filterdPoints.ToVector3s();
                this.EffectivePointCloud = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
            }

            this.Idle();
        }
        #endregion


        //法向量

        #region 估算质心 —— async void EstimateCentroid()
        /// <summary>
        /// 估算质心
        /// </summary>
        public async void EstimateCentroid()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
            Point3F centroid = await Task.Run(() => this._cloudNormals.EstimateCentroid(points));
            this.EffectiveCentroid = new[] { centroid }.ToPointGeometry3D();

            this.Idle();
        }
        #endregion

        #region K估算法向量 —— async void EstimateNormalsByK()
        /// <summary>
        /// K估算法向量
        /// </summary>
        public async void EstimateNormalsByK()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理法向量
            this.EffectiveNormals.Clear();

            KBasedViewModel viewModel = ResolveMediator.Resolve<KBasedViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                Point3F[] points = this.EffectivePointCloud.Points.ToPoint3Fs().ToArray();
                Normal3F[] normals = await Task.Run(() => this._cloudNormals.EstimateNormalsByK(points, viewModel.K!.Value));
                for (int i = 0; i < points.Length; i++)
                {
                    Point3F point = points[i];
                    Normal3F normal = normals[i];

                    LineGeometry3D lineGeometry3D = normal.ToLineGeometry3D(point, viewModel.NormalLength!.Value);
                    LineGeometryModel3D lineGeometryModel3D = new LineGeometryModel3D();
                    lineGeometryModel3D.Geometry = lineGeometry3D;
                    lineGeometryModel3D.Color = viewModel.NormalColor!.Value;
                    lineGeometryModel3D.Thickness = viewModel.NormalThickness!.Value;
                    this.EffectiveNormals.Add(lineGeometryModel3D);
                }
            }

            this.Idle();
        }
        #endregion

        #region K估算法向量(OMP) —— async void EstimateNormalsByKP()
        /// <summary>
        /// K估算法向量(OMP)
        /// </summary>
        public async void EstimateNormalsByKP()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理法向量
            this.EffectiveNormals.Clear();

            KBasedViewModel viewModel = ResolveMediator.Resolve<KBasedViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                Point3F[] points = this.EffectivePointCloud.Points.ToPoint3Fs().ToArray();
                Normal3F[] normals = await Task.Run(() => this._cloudNormals.EstimateNormalsByKP(points, viewModel.K!.Value));
                for (int i = 0; i < points.Length; i++)
                {
                    Point3F point = points[i];
                    Normal3F normal = normals[i];

                    LineGeometry3D lineGeometry3D = normal.ToLineGeometry3D(point, viewModel.NormalLength!.Value);
                    LineGeometryModel3D lineGeometryModel3D = new LineGeometryModel3D();
                    lineGeometryModel3D.Geometry = lineGeometry3D;
                    lineGeometryModel3D.Color = viewModel.NormalColor!.Value;
                    lineGeometryModel3D.Thickness = viewModel.NormalThickness!.Value;
                    this.EffectiveNormals.Add(lineGeometryModel3D);
                }
            }

            this.Idle();
        }
        #endregion

        #region Radius估算法向量 —— async void EstimateNormalsByRadius()
        /// <summary>
        /// Radius估算法向量
        /// </summary>
        public async void EstimateNormalsByRadius()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理法向量
            this.EffectiveNormals.Clear();

            RadiusBasedViewModel viewModel = ResolveMediator.Resolve<RadiusBasedViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                Point3F[] points = this.EffectivePointCloud.Points.ToPoint3Fs().ToArray();
                Normal3F[] normals = await Task.Run(() => this._cloudNormals.EstimateNormalsByRadius(points, viewModel.Radius!.Value));
                for (int i = 0; i < points.Length; i++)
                {
                    Point3F point = points[i];
                    Normal3F normal = normals[i];

                    LineGeometry3D lineGeometry3D = normal.ToLineGeometry3D(point, viewModel.NormalLength!.Value);
                    LineGeometryModel3D lineGeometryModel3D = new LineGeometryModel3D();
                    lineGeometryModel3D.Geometry = lineGeometry3D;
                    lineGeometryModel3D.Color = viewModel.NormalColor!.Value;
                    lineGeometryModel3D.Thickness = viewModel.NormalThickness!.Value;
                    this.EffectiveNormals.Add(lineGeometryModel3D);
                }
            }

            this.Idle();
        }
        #endregion

        #region Radius估算法向量(OMP) —— async void EstimateNormalsByRadiusP()
        /// <summary>
        /// Radius估算法向量(OMP)
        /// </summary>
        public async void EstimateNormalsByRadiusP()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理法向量
            this.EffectiveNormals.Clear();

            RadiusBasedViewModel viewModel = ResolveMediator.Resolve<RadiusBasedViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                Point3F[] points = this.EffectivePointCloud.Points.ToPoint3Fs().ToArray();
                Normal3F[] normals = await Task.Run(() => this._cloudNormals.EstimateNormalsByRadiusP(points, viewModel.Radius!.Value));
                for (int i = 0; i < points.Length; i++)
                {
                    Point3F point = points[i];
                    Normal3F normal = normals[i];

                    LineGeometry3D lineGeometry3D = normal.ToLineGeometry3D(point, viewModel.NormalLength!.Value);
                    LineGeometryModel3D lineGeometryModel3D = new LineGeometryModel3D();
                    lineGeometryModel3D.Geometry = lineGeometry3D;
                    lineGeometryModel3D.Color = viewModel.NormalColor!.Value;
                    lineGeometryModel3D.Thickness = viewModel.NormalThickness!.Value;
                    this.EffectiveNormals.Add(lineGeometryModel3D);
                }
            }

            this.Idle();
        }
        #endregion


        //关键点

        #region 计算NARF关键点 —— async void ComputeNARF()
        /// <summary>
        /// 计算NARF关键点
        /// </summary>
        public async void ComputeNARF()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理关键点
            this.EffectiveKeyPoints = null;

            NarfViewModel viewModel = ResolveMediator.Resolve<NarfViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] keyPoints = await Task.Run(() => this._cloudKeyPoints.ComputeNARF(points, viewModel.AngularResolution!.Value, viewModel.MaxAngleWidth!.Value, viewModel.MaxAngleHeight!.Value, viewModel.NoiseLevel!.Value, viewModel.MinRange!.Value, viewModel.BorderSize!.Value, viewModel.SupportSize!.Value));

                IEnumerable<Vector3> positions = keyPoints.ToVector3s();
                this.EffectiveKeyPoints = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion

        #region 计算ISS关键点 —— async void ComputeISS()
        /// <summary>
        /// 计算ISS关键点
        /// </summary>
        public async void ComputeISS()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理关键点
            this.EffectiveKeyPoints = null;

            IssViewModel viewModel = ResolveMediator.Resolve<IssViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] keyPoints = await Task.Run(() => this._cloudKeyPoints.ComputeISS(points, viewModel.SalientRadius!.Value, viewModel.NonMaxRadius!.Value, viewModel.Threshold21!.Value, viewModel.Threshold32!.Value, viewModel.MinNeighborsCount!.Value, viewModel.ThreadsCount!.Value));

                IEnumerable<Vector3> positions = keyPoints.ToVector3s();
                this.EffectiveKeyPoints = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion

        #region 计算SIFT关键点 —— async void ComputeSIFT()
        /// <summary>
        /// 计算SIFT关键点
        /// </summary>
        public async void ComputeSIFT()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理关键点
            this.EffectiveKeyPoints = null;

            SiftViewModel viewModel = ResolveMediator.Resolve<SiftViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] keyPoints = await Task.Run(() => this._cloudKeyPoints.ComputeSIFT(points, viewModel.MinScale!.Value, viewModel.OctavesCount!.Value, viewModel.ScalesPerOctaveCount!.Value, viewModel.MinContrast!.Value));

                IEnumerable<Vector3> positions = keyPoints.ToVector3s();
                this.EffectiveKeyPoints = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion

        #region 计算Harris关键点 —— async void ComputeHarris()
        /// <summary>
        /// 计算Harris关键点
        /// </summary>
        public async void ComputeHarris()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理关键点
            this.EffectiveKeyPoints = null;

            HarrisViewModel viewModel = ResolveMediator.Resolve<HarrisViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] keyPoints = await Task.Run(() => this._cloudKeyPoints.ComputeHarris(points, viewModel.NonMaxSupression!.Value, viewModel.Radius!.Value, viewModel.Threshold!.Value));

                IEnumerable<Vector3> positions = keyPoints.ToVector3s();
                this.EffectiveKeyPoints = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion

        #region 计算SUSAN关键点 —— async void ComputeSUSAN()
        /// <summary>
        /// 计算SUSAN关键点
        /// </summary>
        public async void ComputeSUSAN()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理关键点
            this.EffectiveKeyPoints = null;

            SusanViewModel viewModel = ResolveMediator.Resolve<SusanViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Point3F[] keyPoints = await Task.Run(() => this._cloudKeyPoints.ComputeSUSAN(points, viewModel.NonMaxSupression!.Value, viewModel.Radius!.Value, viewModel.DistanceThreshold!.Value, viewModel.AngularThreshold!.Value, viewModel.IntensityThreshold!.Value));

                IEnumerable<Vector3> positions = keyPoints.ToVector3s();
                this.EffectiveKeyPoints = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion


        //事件

        #region 键盘按下事件 —— void OnKeyDown()
        /// <summary>
        /// 键盘按下事件
        /// </summary>
        public void OnKeyDown()
        {
            if (Keyboard.IsKeyDown(Key.F5))
            {
                #region # 验证

                if (this.EffectivePointCloud == null)
                {
                    MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                    return;
                }

                #endregion

                this.RefreshCloud();
            }
            if (Keyboard.IsKeyDown(Key.LeftCtrl) && Keyboard.IsKeyDown(Key.S))
            {
                #region # 验证

                if (this.EffectivePointCloud == null)
                {
                    MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                    return;
                }

                #endregion

                this.SaveCloud();
            }
        }
        #endregion


        //Private

        #region 加载点云 —— async Task ReloadCloud()
        /// <summary>
        /// 加载点云
        /// </summary>
        public async Task ReloadCloud()
        {
            if (this.CloudColorType == PointColorType.Mono)
            {
                await this.ReloadMonoCloud();
            }
            if (this.CloudColorType == PointColorType.RGBA)
            {
                await this.ReloadColorCloud();
            }
        }
        #endregion

        #region 加载灰度点云 —— async Task ReloadMonoCloud()
        /// <summary>
        /// 加载灰度点云
        /// </summary>
        private async Task ReloadMonoCloud()
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(this.FilePath))
            {
                return;
            }
            if (string.IsNullOrWhiteSpace(this.FileExtension))
            {
                return;
            }

            #endregion

            Point3F[] points = this.FileExtension switch
            {
                Constants.PCD => await Task.Run(() => this._cloudFiles.LoadPCD(this.FilePath)),
                Constants.PLY => await Task.Run(() => this._cloudFiles.LoadPLY(this.FilePath)),
                Constants.OBJ => await Task.Run(() => this._cloudFiles.LoadOBJ(this.FilePath)),
                _ => throw new NotSupportedException("不支持的点云格式！")
            };

            //过滤NaN
            points = points.FilterNaN();

            this.OriginalPointCloud = points.ToPointGeometry3D();
            this.EffectivePointCloud = points.ToPointGeometry3D();

            //随机颜色
            this.PointColor = ColorExtension.RandomColor();

            //清理质心、法向量、关键点
            this.EffectiveCentroid = null;
            this.EffectiveNormals.Clear();
            this.EffectiveKeyPoints = null;
        }
        #endregion

        #region 加载彩色点云 —— async Task ReloadColorCloud()
        /// <summary>
        /// 加载彩色点云
        /// </summary>
        private async Task ReloadColorCloud()
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(this.FilePath))
            {
                return;
            }
            if (string.IsNullOrWhiteSpace(this.FileExtension))
            {
                return;
            }

            #endregion

            Point3Color4[] pointColors = this.FileExtension switch
            {
                Constants.PCD => await Task.Run(() => this._cloudFiles.LoadColorPCD(this.FilePath)),
                Constants.PLY => await Task.Run(() => this._cloudFiles.LoadColorPLY(this.FilePath)),
                Constants.OBJ => await Task.Run(() => this._cloudFiles.LoadColorOBJ(this.FilePath)),
                _ => throw new NotSupportedException("不支持的点云格式！")
            };

            //过滤NaN
            pointColors = pointColors.FilterNaN();

            this.OriginalPointCloud = pointColors.ToPointGeometry3D();
            this.EffectivePointCloud = pointColors.ToPointGeometry3D();

            //白底
            this.PointColor = Colors.White;

            //清理质心、法向量、关键点
            this.EffectiveCentroid = null;
            this.EffectiveNormals.Clear();
            this.EffectiveKeyPoints = null;
        }
        #endregion

        #endregion
    }
}
