using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SharpDX;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace PCLSharp.Client.ViewModels.CommonContext
{
    /// <summary>
    /// 仿射变换视图模型
    /// </summary>
    public class AffineViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云通用操作接口;
        /// </summary>
        private readonly ICloudCommon _cloudCommon;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public AffineViewModel(ICloudCommon cloudCommon)
            : base(cloudCommon)
        {
            this._cloudCommon = cloudCommon;
        }

        #endregion

        #region # 属性

        #region X轴位置 —— float? X
        /// <summary>
        /// X轴位置
        /// </summary>
        [DependencyProperty]
        public float? X { get; set; }
        #endregion

        #region Y轴位置 —— float? Y
        /// <summary>
        /// Y轴位置
        /// </summary>
        [DependencyProperty]
        public float? Y { get; set; }
        #endregion

        #region Z轴位置 —— float? Z
        /// <summary>
        /// Z轴位置
        /// </summary>
        [DependencyProperty]
        public float? Z { get; set; }
        #endregion

        #region X轴旋转角度 —— float? RX
        /// <summary>
        /// X轴旋转角度
        /// </summary>
        [DependencyProperty]
        public float? RX { get; set; }
        #endregion

        #region Y轴旋转角度 —— float? RY
        /// <summary>
        /// Y轴旋转角度
        /// </summary>
        [DependencyProperty]
        public float? RY { get; set; }
        #endregion

        #region Z轴旋转角度 —— float? RZ
        /// <summary>
        /// Z轴旋转角度
        /// </summary>
        [DependencyProperty]
        public float? RZ { get; set; }
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
            this.X = 0.1f;
            this.Y = 0.1f;
            this.Z = 0.1f;
            this.RX = 15;
            this.RY = 15;
            this.RZ = 0;

            return base.OnInitializeAsync(cancellationToken);
        }
        #endregion

        #region 应用 —— async void Apply()
        /// <summary>
        /// 应用
        /// </summary>
        public async void Apply()
        {
            #region # 验证

            if (!this.X.HasValue)
            {
                MessageBox.Show("X轴位置不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Y.HasValue)
            {
                MessageBox.Show("Y轴位置不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Z.HasValue)
            {
                MessageBox.Show("Z轴位置不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.RX.HasValue)
            {
                MessageBox.Show("X轴旋转角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.RY.HasValue)
            {
                MessageBox.Show("Y轴旋转角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.RZ.HasValue)
            {
                MessageBox.Show("Z轴旋转角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.PointCloud == null)
            {
                MessageBox.Show("点云不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            IEnumerable<Point3F> points = this.BasedPointCloud.Points.ToPoint3Fs();
            Pose pose = new Pose(this.X!.Value, this.Y!.Value, this.Z!.Value, this.RX!.Value, this.RY!.Value, this.RZ!.Value);
            Point3F[] transformedPoints = await Task.Run(() => this._cloudCommon.AffineTransform(points, pose));

            IEnumerable<Vector3> positions = transformedPoints.ToVector3s();
            this.PointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(positions)
            };

            this.Idle();
        }
        #endregion

        #endregion
    }
}
