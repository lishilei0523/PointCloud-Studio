using Caliburn.Micro;
using HelixToolkit.Wpf.SharpDX;
using Microsoft.Win32;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Extensions;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using SD.IOC.Core.Mediators;
using SharpDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
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
        /// 点云搜索接口
        /// </summary>
        private readonly ICloudSearch _cloudSearch;

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
        /// 点云特征接口
        /// </summary>
        private readonly ICloudFeatures _cloudFeatures;

        /// <summary>
        /// 点云分割接口
        /// </summary>
        private readonly ICloudSegmentations _cloudSegmentations;

        /// <summary>
        /// 窗体管理器
        /// </summary>
        private readonly IWindowManager _windowManager;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public IndexViewModel(ICloudCommon cloudCommon, ICloudFiles cloudFiles, ICloudSearch cloudSearch, ICloudFilters cloudFilters, ICloudNormals cloudNormals, ICloudKeyPoints cloudKeyPoints, ICloudFeatures cloudFeatures, ICloudSegmentations cloudSegmentations, IWindowManager windowManager)
        {
            this._cloudCommon = cloudCommon;
            this._cloudFiles = cloudFiles;
            this._cloudSearch = cloudSearch;
            this._cloudFilters = cloudFilters;
            this._cloudNormals = cloudNormals;
            this._cloudKeyPoints = cloudKeyPoints;
            this._cloudFeatures = cloudFeatures;
            this._cloudSegmentations = cloudSegmentations;
            this._windowManager = windowManager;
        }

        #endregion

        #region # 属性

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

        #region 源点云 —— PointGeometry3D SourceCloud
        /// <summary>
        /// 源点云
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

        #region 源点云质心 —— PointGeometry3D SourceCentroid
        /// <summary>
        /// 源点云质心
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D SourceCentroid { get; set; }
        #endregion

        #region 相机 —— PerspectiveCamera Camera
        /// <summary>
        /// 相机
        /// </summary>
        [DependencyProperty]
        public PerspectiveCamera Camera { get; set; }
        #endregion

        #region 参数视图模型 —— ParamViewModel ParamViewModel
        /// <summary>
        /// 参数视图模型
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
            //初始化相机
            this.ResetCamera();

            return base.OnInitializeAsync(cancellationToken);
        }
        #endregion


        //Actions

        #region 打开源点云 —— async void OpenSourceCloud()
        /// <summary>
        /// 打开源点云
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
            ParamViewModel paramViewModel = ResolveMediator.Resolve<ParamViewModel>();
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
            //TODO 实现
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


        //Private

        #region 加载源点云 —— async Task ReloadSourceCloud()
        /// <summary>
        /// 加载源点云
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
