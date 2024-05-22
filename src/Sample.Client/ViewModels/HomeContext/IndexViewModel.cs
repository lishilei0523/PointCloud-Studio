using Caliburn.Micro;
using HelixToolkit.Wpf.SharpDX;
using Microsoft.Win32;
using PCLSharp.FileIO.Interfaces;
using PCLSharp.Filters.Interfaces;
using PCLSharp.Normals.Interfaces;
using PCLSharp.Primitives.Models;
using Sample.Client.ViewModels.FilterContext;
using Sample.Presentation.Maps;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
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
using Constants = PCLSharp.Constants;
using PerspectiveCamera = HelixToolkit.Wpf.SharpDX.PerspectiveCamera;

namespace Sample.Client.ViewModels.HomeContext
{
    /// <summary>
    /// 首页视图模型
    /// </summary>
    public class IndexViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云读写器接口
        /// </summary>
        private readonly ICloudConductor _cloudConductor;

        /// <summary>
        /// 点云滤波接口
        /// </summary>
        private readonly ICloudFilters _cloudFilters;

        /// <summary>
        /// 点云法向量接口
        /// </summary>
        private readonly ICloudNormals _cloudNormals;

        /// <summary>
        /// 窗体管理器
        /// </summary>
        private readonly IWindowManager _windowManager;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public IndexViewModel(ICloudConductor cloudConductor, ICloudFilters cloudFilters, ICloudNormals cloudNormals, IWindowManager windowManager)
        {
            this._cloudConductor = cloudConductor;
            this._cloudFilters = cloudFilters;
            this._cloudNormals = cloudNormals;
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

        #region 原始点云颜色 —— Color OriginalPointColor
        /// <summary>
        /// 原始点云颜色
        /// </summary>
        [DependencyProperty]
        public Color OriginalPointColor { get; set; }
        #endregion

        #region 效果点云颜色 —— Color EffectivePointColor
        /// <summary>
        /// 效果点云颜色
        /// </summary>
        [DependencyProperty]
        public Color EffectivePointColor { get; set; }
        #endregion

        #region 效果点云法向量列表 —— ObservableCollection<LineGeometryModel3D> EffectivePointNormals
        /// <summary>
        /// 效果点云法向量列表
        /// </summary>
        [DependencyProperty]
        public ObservableCollection<LineGeometryModel3D> EffectivePointNormals { get; set; }
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
            this.EffectivePointNormals = new ObservableCollection<LineGeometryModel3D>();

            //初始化相机
            this.Camera = new PerspectiveCamera
            {
                LookDirection = new Vector3D(0, 0, -2),
                UpDirection = new Vector3D(0, 1, 0),
                Position = new Point3D(0, 0, 2),
                NearPlaneDistance = 0.125,
                FarPlaneDistance = double.PositiveInfinity,
                FieldOfView = 30
            };

            return base.OnInitializeAsync(cancellationToken);
        }
        #endregion


        //Actions

        #region 打开点云 —— async void OpenCloud()
        /// <summary>
        /// 打开点云
        /// </summary>
        public async void OpenCloud()
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
                await this.ReloadCloud();

                this.Idle();
            }
        }
        #endregion

        #region 刷新点云 —— async void RefreshCloud()
        /// <summary>
        /// 刷新点云
        /// </summary>
        public async void RefreshCloud()
        {
            this.Busy();

            await this.ReloadCloud();

            this.Idle();
        }
        #endregion

        #region 保存点云 —— async void SaveCloud()
        /// <summary>
        /// 保存点云
        /// </summary>
        public async void SaveCloud()
        {
            this.Busy();

            IEnumerable<Point3F> points = this.EffectivePointCloud.Points.Select(x => x.ToPoint3F());
            if (this.FileExtension == Constants.PCD)
            {
                await Task.Run(() => this._cloudConductor.SaveTextPCD(points, this.FilePath));
            }
            if (this.FileExtension == Constants.PLY)
            {
                await Task.Run(() => this._cloudConductor.SaveTextPLY(points, this.FilePath));
            }
            if (this.FileExtension == Constants.OBJ)
            {
                this.FilePath = this.FilePath.Replace(Constants.OBJ, Constants.PCD);
                await Task.Run(() => this._cloudConductor.SaveTextPCD(points, this.FilePath));
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

                IEnumerable<Point3F> points = this.EffectivePointCloud.Points.Select(x => x.ToPoint3F());
                string filePath = saveFileDialog.FileName;
                string fileExt = Path.GetExtension(filePath);
                if (fileExt == Constants.PCD)
                {
                    await Task.Run(() => this._cloudConductor.SaveTextPCD(points, filePath));
                }
                if (fileExt == Constants.PLY)
                {
                    await Task.Run(() => this._cloudConductor.SaveTextPLY(points, filePath));
                }

                this.Idle();
                this.ToastSuccess("保存成功！");
            }
        }
        #endregion

        #region 适用直通滤波 —— async void ApplyPassThrogh()
        /// <summary>
        /// 适用直通滤波
        /// </summary>
        public async void ApplyPassThrogh()
        {
            #region # 验证

            if (this.OriginalPointCloud == null)
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
                IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
                ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyPassThrogh(points, viewModel.SelectedAxis, viewModel.LimitMin!.Value, viewModel.LimitMax!.Value));

                IEnumerable<Vector3> positions = filterdPoints.Select(x => x.ToVector3());
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

            if (this.OriginalPointCloud == null)
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
                IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
                ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyRandomSampling(points, viewModel.Seed!.Value, viewModel.SamplesCount!.Value));

                IEnumerable<Vector3> positions = filterdPoints.Select(x => x.ToVector3());
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

            if (this.OriginalPointCloud == null)
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
                IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
                ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyUniformSampling(points, viewModel.Radius!.Value));

                IEnumerable<Vector3> positions = filterdPoints.Select(x => x.ToVector3());
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

            if (this.OriginalPointCloud == null)
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
                IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
                ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyVoxelGrid(points, viewModel.LeafSize!.Value));

                IEnumerable<Vector3> positions = filterdPoints.Select(x => x.ToVector3());
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

            if (this.OriginalPointCloud == null)
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
                IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
                ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyApproximateVoxelGrid(points, viewModel.LeafSize!.Value));

                IEnumerable<Vector3> positions = filterdPoints.Select(x => x.ToVector3());
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

            if (this.OriginalPointCloud == null)
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
                IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
                ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyStatisticalOutlierRemoval(points, viewModel.MeanK!.Value, viewModel.StddevMult!.Value));

                IEnumerable<Vector3> positions = filterdPoints.Select(x => x.ToVector3());
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

            if (this.OriginalPointCloud == null)
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
                IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
                ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyRadiusOutlierRemoval(points, viewModel.Radius!.Value, viewModel.MinNeighborsInRadius!.Value));

                IEnumerable<Vector3> positions = filterdPoints.Select(x => x.ToVector3());
                this.EffectivePointCloud = new PointGeometry3D
                {
                    Positions = new Vector3Collection(positions)
                };
            }

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

            Point3F[] points = this.EffectivePointCloud.Points.Select(point => point.ToPoint3F()).ToArray();
            Normal3F[] normal3Fs = await Task.Run(() => this._cloudNormals.EstimateNormalsByK(points, 5));
            for (int i = 0; i < points.Length; i++)
            {
                Point3F point = points[i];
                Normal3F normal = normal3Fs[i];

                LineGeometry3D lineGeometry3D = normal.ToLineGeometry3D(point, 0.01f);
                LineGeometryModel3D lineGeometryModel3D = new LineGeometryModel3D();
                lineGeometryModel3D.Geometry = lineGeometry3D;
                lineGeometryModel3D.Color = Colors.Red;
                lineGeometryModel3D.Thickness = 0.5;
                this.EffectivePointNormals.Add(lineGeometryModel3D);
            }

            this.Idle();
        }
        #endregion

        #region 键盘按下事件 —— void OnKeyDown()
        /// <summary>
        /// 键盘按下事件
        /// </summary>
        public void OnKeyDown()
        {
            if (Keyboard.IsKeyDown(Key.F5))
            {
                this.RefreshCloud();
            }
            if (Keyboard.IsKeyDown(Key.LeftCtrl) && Keyboard.IsKeyDown(Key.S))
            {
                this.SaveCloud();
            }
        }
        #endregion


        //Private

        #region 加载点云 —— async Task ReloadCloud()
        /// <summary>
        /// 加载点云
        /// </summary>
        private async Task ReloadCloud()
        {
            Point3F[] points = this.FileExtension switch
            {
                Constants.PCD => await Task.Run(() => this._cloudConductor.LoadPCD(this.FilePath)),
                Constants.PLY => await Task.Run(() => this._cloudConductor.LoadPLY(this.FilePath)),
                Constants.OBJ => await Task.Run(() => this._cloudConductor.LoadOBJ(this.FilePath)),
                _ => throw new NotSupportedException("不支持的点云格式！")
            };

            Vector3[] vectors = points.Select(x => x.ToVector3()).ToArray();
            this.OriginalPointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(vectors)
            };
            this.EffectivePointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(vectors)
            };

            Random random = new Random((int)DateTime.Now.Ticks);
            byte r = (byte)random.Next(0, 255);
            byte g = (byte)random.Next(0, 255);
            byte b = (byte)random.Next(0, 255);
            this.OriginalPointColor = Color.FromRgb(r, g, b);
            this.EffectivePointColor = Color.FromRgb(r, g, b);

            //清理法向量
            this.EffectivePointNormals.Clear();
        }
        #endregion

        #endregion
    }
}
