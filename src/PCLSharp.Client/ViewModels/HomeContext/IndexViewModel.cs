using Caliburn.Micro;
using HelixToolkit.Wpf.SharpDX;
using Microsoft.Win32;
using PCLSharp.Client.ViewModels.CommonContext;
using PCLSharp.Client.ViewModels.FeatureContext;
using PCLSharp.Client.ViewModels.FilterContext;
using PCLSharp.Client.ViewModels.KeyPointContext;
using PCLSharp.Client.ViewModels.NormalContext;
using PCLSharp.Client.ViewModels.SearchContext;
using PCLSharp.Client.ViewModels.SegmentationContext;
using PCLSharp.Extensions.Helix;
using PCLSharp.Extensions.Plotter;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Constants;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Features;
using PCLSharp.Primitives.Models;
using ScottPlot;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using SD.Infrastructure.WPF.Extensions;
using SD.IOC.Core.Mediators;
using SharpDX;
using SkiaSharp;
using SkiaSharp.Views.WPF;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using Color = System.Windows.Media.Color;
using Colors = System.Windows.Media.Colors;
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
        /// 点云通用操作接口;
        /// </summary>
        private readonly ICloudCommon _cloudCommon;

        /// <summary>
        /// 点云文件接口
        /// </summary>
        private readonly ICloudFiles _cloudFiles;

        /// <summary>
        /// 点云特征接口
        /// </summary>
        private readonly ICloudFeatures _cloudFeatures;

        /// <summary>
        /// 窗体管理器
        /// </summary>
        private readonly IWindowManager _windowManager;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public IndexViewModel(ICloudCommon cloudCommon, ICloudFiles cloudFiles, ICloudFeatures cloudFeatures, IWindowManager windowManager)
        {
            this._cloudCommon = cloudCommon;
            this._cloudFiles = cloudFiles;
            this._cloudFeatures = cloudFeatures;
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

        #region 文件路径 —— string FilePath
        /// <summary>
        /// 文件路径
        /// </summary>
        public string FilePath { get; set; }
        #endregion

        #region 文件格式 —— string FileExtension
        /// <summary>
        /// 文件格式
        /// </summary>
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

        #region 效果点云法向量 —— ObservableCollection<LineGeometryModel3D> EffectiveNormals
        /// <summary>
        /// 效果点云法向量
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
            this.MSAALevel = MSAALevel.Maximum;
            this.HighImageQuality = true;
            this.LabelColor = Colors.Black;
            this.BackgroundColor = Colors.LightGray;
            this.KeyPointColor = Colors.Red;
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
                centroid = await Task.Run(() => this._cloudCommon.EstimateCentroid(points));
                this.EffectiveCentroid = new[] { centroid }.ToPointGeometry3D();
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


        //常用

        #region 切换画质 —— void SwitchImageQuality()
        /// <summary>
        /// 切换画质
        /// </summary>
        public void SwitchImageQuality()
        {
            this.MSAALevel = this.HighImageQuality ? MSAALevel.Maximum : MSAALevel.Disable;
        }
        #endregion

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
            Point3F centroid = await Task.Run(() => this._cloudCommon.EstimateCentroid(points));
            this.EffectiveCentroid = new[] { centroid }.ToPointGeometry3D();

            this.Idle();
        }
        #endregion

        #region 仿射变换 —— async void AffineTransform()
        /// <summary>
        /// 仿射变换
        /// </summary>
        public async void AffineTransform()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            AffineViewModel viewModel = ResolveMediator.Resolve<AffineViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion

        #region 盒子裁剪 —— async void CropBox()
        /// <summary>
        /// 盒子裁剪
        /// </summary>
        public async void CropBox()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            CropBoxViewModel viewModel = ResolveMediator.Resolve<CropBoxViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion

        #region 投射平面 —— async void ProjectPlane()
        /// <summary>
        /// 投射平面
        /// </summary>
        public async void ProjectPlane()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            ProjectViewModel viewModel = ResolveMediator.Resolve<ProjectViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion

        #region 提取边框 —— async void ExtractBorder()
        /// <summary>
        /// 提取边框
        /// </summary>
        public async void ExtractBorder()
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
            Point3F[] borderPoints = await Task.Run(() => this._cloudCommon.ExtractBorder(points));

            IEnumerable<Vector3> positions = borderPoints.ToVector3s();
            this.EffectivePointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(positions)
            };

            this.Idle();
        }
        #endregion

        #region 提取边界 —— async void ExtractBoundary()
        /// <summary>
        /// 提取边界
        /// </summary>
        public async void ExtractBoundary()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            BoundaryViewModel viewModel = ResolveMediator.Resolve<BoundaryViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion

        #region 技术支持 —— void Support()
        /// <summary>
        /// 技术支持
        /// </summary>
        public void Support()
        {
            Process.Start("https://gitee.com/lishilei0523/PointCloud-Studio");
        }
        #endregion


        //搜索

        #region K近邻搜索 —— async void KSearch()
        /// <summary>
        /// K近邻搜索
        /// </summary>
        public async void KSearch()
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

            KSearchViewModel viewModel = ResolveMediator.Resolve<KSearchViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveKeyPoints = viewModel.KeyPoints;
                this.KeyPointColor = this.PointColor.Invert();
            }

            this.Idle();
        }
        #endregion

        #region 半径搜索 —— async void RadiusSearch()
        /// <summary>
        /// 半径搜索
        /// </summary>
        public async void RadiusSearch()
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

            RadiusSearchViewModel viewModel = ResolveMediator.Resolve<RadiusSearchViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveKeyPoints = viewModel.KeyPoints;
                this.KeyPointColor = this.PointColor.Invert();
            }

            this.Idle();
        }
        #endregion

        #region 八叉树搜索 —— async void OctreeSearch()
        /// <summary>
        /// 八叉树搜索
        /// </summary>
        public async void OctreeSearch()
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

            OctreeSearchViewModel viewModel = ResolveMediator.Resolve<OctreeSearchViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveKeyPoints = viewModel.KeyPoints;
                this.KeyPointColor = this.PointColor.Invert();
            }

            this.Idle();
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion

        #region 适用近似体素降采样 —— async void ApplyApproxVoxelGrid()
        /// <summary>
        /// 适用近似体素降采样
        /// </summary>
        public async void ApplyApproxVoxelGrid()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            ApproxVoxelGridViewModel viewModel = ResolveMediator.Resolve<ApproxVoxelGridViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion


        //法向量

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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveNormals = viewModel.Normals;
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveNormals = viewModel.Normals;
            }

            this.Idle();
        }
        #endregion


        //关键点

        #region 检测NARF关键点 —— async void DetectNARF()
        /// <summary>
        /// 检测NARF关键点
        /// </summary>
        public async void DetectNARF()
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

            KeyPointContext.NarfViewModel viewModel = ResolveMediator.Resolve<KeyPointContext.NarfViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveKeyPoints = viewModel.KeyPoints;
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion

        #region 检测ISS关键点 —— async void DetectISS()
        /// <summary>
        /// 检测ISS关键点
        /// </summary>
        public async void DetectISS()
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveKeyPoints = viewModel.KeyPoints;
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion

        #region 检测SIFT关键点 —— async void DetectSIFT()
        /// <summary>
        /// 检测SIFT关键点
        /// </summary>
        public async void DetectSIFT()
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveKeyPoints = viewModel.KeyPoints;
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion

        #region 检测Harris关键点 —— async void DetectHarris()
        /// <summary>
        /// 检测Harris关键点
        /// </summary>
        public async void DetectHarris()
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveKeyPoints = viewModel.KeyPoints;
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion

        #region 检测SUSAN关键点 —— async void DetectSUSAN()
        /// <summary>
        /// 检测SUSAN关键点
        /// </summary>
        public async void DetectSUSAN()
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
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectiveKeyPoints = viewModel.KeyPoints;
                this.KeyPointColor = viewModel.KeyPointColor!.Value;
            }

            this.Idle();
        }
        #endregion


        //特征

        #region 计算NARF特征 —— async void ComputeNARF()
        /// <summary>
        /// 计算NARF特征
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

            FeatureContext.NarfViewModel viewModel = ResolveMediator.Resolve<FeatureContext.NarfViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Narf36F[] descriptors = await Task.Run(() => this._cloudFeatures.ComputeNARF(points, viewModel.AngularResolution!.Value, viewModel.MaxAngleWidth!.Value, viewModel.MaxAngleHeight!.Value, viewModel.NoiseLevel!.Value, viewModel.MinRange!.Value, viewModel.BorderSize!.Value, viewModel.SupportSize!.Value, viewModel.RotationInvariant!.Value));

                //绘制直方图
                using Plot plot = new Plot();
                await Task.Run(() => plot.AddNARF(descriptors));
                using SKImage skImage = await Task.Run(() => plot.GetSKImage(viewModel.ImageWidth!.Value, viewModel.ImageHeight!.Value));
                BitmapSource bitmapSource = skImage.ToWriteableBitmap();

                ImageViewModel imageViewModel = ResolveMediator.Resolve<ImageViewModel>();
                imageViewModel.Load(bitmapSource, "NARF特征");
                await this._windowManager.ShowDialogAsync(imageViewModel);
            }

            this.Idle();
        }
        #endregion

        #region 计算PFH特征 —— async void ComputePFH()
        /// <summary>
        /// 计算PFH特征
        /// </summary>
        public async void ComputePFH()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            PfhViewModel viewModel = ResolveMediator.Resolve<PfhViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                PFHSignature125F[] descriptors = await Task.Run(() => this._cloudFeatures.ComputePFH(points, viewModel.NormalK!.Value, viewModel.FeatureK!.Value, viewModel.ThreadsCount!.Value));

                //绘制直方图
                using Plot plot = new Plot();
                await Task.Run(() => plot.AddPFH(descriptors));
                using SKImage skImage = await Task.Run(() => plot.GetSKImage(viewModel.ImageWidth!.Value, viewModel.ImageHeight!.Value));
                BitmapSource bitmapSource = skImage.ToWriteableBitmap();

                ImageViewModel imageViewModel = ResolveMediator.Resolve<ImageViewModel>();
                imageViewModel.Load(bitmapSource, "PFH特征");
                await this._windowManager.ShowDialogAsync(imageViewModel);
            }

            this.Idle();
        }
        #endregion

        #region 计算FPFH特征 —— async void ComputeFPFH()
        /// <summary>
        /// 计算FPFH特征
        /// </summary>
        public async void ComputeFPFH()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            PfhViewModel viewModel = ResolveMediator.Resolve<PfhViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                FPFHSignature33F[] descriptors = await Task.Run(() => this._cloudFeatures.ComputeFPFH(points, viewModel.NormalK!.Value, viewModel.FeatureK!.Value, viewModel.ThreadsCount!.Value));

                //绘制直方图
                using Plot plot = new Plot();
                await Task.Run(() => plot.AddFPFH(descriptors));
                using SKImage skImage = await Task.Run(() => plot.GetSKImage(viewModel.ImageWidth!.Value, viewModel.ImageHeight!.Value));
                BitmapSource bitmapSource = skImage.ToWriteableBitmap();

                ImageViewModel imageViewModel = ResolveMediator.Resolve<ImageViewModel>();
                imageViewModel.Load(bitmapSource, "FPFH特征");
                await this._windowManager.ShowDialogAsync(imageViewModel);
            }

            this.Idle();
        }
        #endregion

        #region 计算3DSC特征 —— async void Compute3DSC()
        /// <summary>
        /// 计算3DSC特征
        /// </summary>
        public async void Compute3DSC()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            Dsc3ViewModel viewModel = ResolveMediator.Resolve<Dsc3ViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                ShapeContext1980F[] descriptors = await Task.Run(() => this._cloudFeatures.Compute3DSC(points, viewModel.NormalK!.Value, viewModel.SearchRadius!.Value, viewModel.PointDensityRadius!.Value, viewModel.MinimalRadius!.Value, viewModel.ThreadsCount!.Value));

                //绘制直方图
                using Plot plot = new Plot();
                await Task.Run(() => plot.Add3DSC(descriptors));
                using SKImage skImage = await Task.Run(() => plot.GetSKImage(viewModel.ImageWidth!.Value, viewModel.ImageHeight!.Value));
                BitmapSource bitmapSource = skImage.ToWriteableBitmap();

                ImageViewModel imageViewModel = ResolveMediator.Resolve<ImageViewModel>();
                imageViewModel.Load(bitmapSource, "3DSC特征");
                await this._windowManager.ShowDialogAsync(imageViewModel);
            }

            this.Idle();
        }
        #endregion

        #region 计算SHOT特征 —— async void ComputeSHOT()
        /// <summary>
        /// 计算SHOT特征
        /// </summary>
        public async void ComputeSHOT()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            ShotViewModel viewModel = ResolveMediator.Resolve<ShotViewModel>();
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.ToPoint3Fs();
                Shot352F[] descriptors = await Task.Run(() => this._cloudFeatures.ComputeSHOT(points, viewModel.NormalK!.Value, viewModel.FeatureRadius!.Value, viewModel.ThreadsCount!.Value));

                //绘制直方图
                using Plot plot = new Plot();
                await Task.Run(() => plot.AddSHOT(descriptors));
                using SKImage skImage = await Task.Run(() => plot.GetSKImage(viewModel.ImageWidth!.Value, viewModel.ImageHeight!.Value));
                BitmapSource bitmapSource = skImage.ToWriteableBitmap();

                ImageViewModel imageViewModel = ResolveMediator.Resolve<ImageViewModel>();
                imageViewModel.Load(bitmapSource, "SHOT特征");
                await this._windowManager.ShowDialogAsync(imageViewModel);
            }

            this.Idle();
        }
        #endregion


        //分割

        #region 分割平面 —— async void SegmentPlane()
        /// <summary>
        /// 分割平面
        /// </summary>
        public async void SegmentPlane()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            PlaneViewModel viewModel = ResolveMediator.Resolve<PlaneViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion

        #region 分割球体 —— async void SegmentSphere()
        /// <summary>
        /// 分割球体
        /// </summary>
        public async void SegmentSphere()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            SphereViewModel viewModel = ResolveMediator.Resolve<SphereViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion

        #region 欧几里得聚类分割 —— async void EuclidClusterSegment()
        /// <summary>
        /// 欧几里得聚类分割
        /// </summary>
        public async void EuclidClusterSegment()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            EuclidViewModel viewModel = ResolveMediator.Resolve<EuclidViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion

        #region 区域生长分割 —— async void RegionGrowingSegment()
        /// <summary>
        /// 区域生长分割
        /// </summary>
        public async void RegionGrowingSegment()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            RegionGrowViewModel viewModel = ResolveMediator.Resolve<RegionGrowViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion

        #region 区域生长颜色分割 —— async void RegionGrowingColorSegment()
        /// <summary>
        /// 区域生长颜色分割
        /// </summary>
        public async void RegionGrowingColorSegment()
        {
            #region # 验证

            if (this.EffectivePointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            RegionGrowColorViewModel viewModel = ResolveMediator.Resolve<RegionGrowColorViewModel>();
            viewModel.Load(this.EffectivePointCloud);
            bool? result = await this._windowManager.ShowDialogAsync(viewModel);
            if (result == true)
            {
                this.EffectivePointCloud = viewModel.PointCloud;
            }

            this.Idle();
        }
        #endregion


        //配准

        #region 配准点云 —— async void AlignCloud()
        /// <summary>
        /// 配准点云
        /// </summary>
        public async void AlignCloud()
        {
            RegistrationContext.IndexViewModel viewModel = ResolveMediator.Resolve<RegistrationContext.IndexViewModel>();
            await this._windowManager.ShowWindowAsync(viewModel);
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

                this.ResetPointCloud();
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
