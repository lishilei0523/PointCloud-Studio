using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using SharpDX;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media.Media3D;
using PerspectiveCamera = HelixToolkit.Wpf.SharpDX.PerspectiveCamera;

namespace PCLSharp.Client.ViewModels.CommonContext
{
    /// <summary>
    /// 点云视图模型基类
    /// </summary>
    public abstract class CloudViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云通用操作接口;
        /// </summary>
        private readonly ICloudCommon _cloudCommon;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        protected CloudViewModel(ICloudCommon cloudCommon)
        {
            this._cloudCommon = cloudCommon;
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

        #region 相机 —— PerspectiveCamera Camera
        /// <summary>
        /// 相机
        /// </summary>
        [DependencyProperty]
        public PerspectiveCamera Camera { get; set; }
        #endregion

        #region 点云 —— PointGeometry3D PointCloud
        /// <summary>
        /// 点云
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D PointCloud { get; set; }
        #endregion

        #region 点云质心 —— PointGeometry3D Centroid
        /// <summary>
        /// 点云质心
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D Centroid { get; set; }
        #endregion

        #region 基准点云 —— PointGeometry3D BasedPointCloud
        /// <summary>
        /// 基准点云
        /// </summary>
        public PointGeometry3D BasedPointCloud { get; set; }
        #endregion

        #endregion

        #region # 方法

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

        #region 加载 —— virtual void Load(PointGeometry3D pointCloud)
        /// <summary>
        /// 加载
        /// </summary>
        public virtual void Load(PointGeometry3D pointCloud)
        {
            this.PointCloud = pointCloud;
            IEnumerable<Vector3> vectors = pointCloud.Points.Select(point => new Vector3(point.P0.X, point.P0.Y, point.P0.Z));
            this.BasedPointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(vectors)
            };
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

            if (this.PointCloud == null)
            {
                MessageBox.Show("点云未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            Point3F centroid;
            if (this.Centroid != null)
            {
                Vector3 position = this.Centroid.Positions.Single();
                centroid = position.ToPoint3F();
            }
            else
            {
                IEnumerable<Point3F> points = this.PointCloud.Points.ToPoint3Fs();
                centroid = await Task.Run(() => this._cloudCommon.EstimateCentroid(points));
                this.Centroid = new[] { centroid }.ToPointGeometry3D();
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
                LookDirection = new Vector3D(0, 0, -200),
                UpDirection = new Vector3D(0, 1, 0),
                Position = new Point3D(0, 0, 200),
                NearPlaneDistance = 0.125,
                FarPlaneDistance = double.PositiveInfinity,
                FieldOfView = 30
            };
        }
        #endregion

        #endregion
    }
}
