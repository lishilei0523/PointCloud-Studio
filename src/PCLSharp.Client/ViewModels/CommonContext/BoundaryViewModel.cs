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
    /// 提取边界视图模型
    /// </summary>
    public class BoundaryViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云通用操作接口;
        /// </summary>
        private readonly ICloudCommon _cloudCommon;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public BoundaryViewModel(ICloudCommon cloudCommon)
            : base(cloudCommon)
        {
            this._cloudCommon = cloudCommon;
        }

        #endregion

        #region # 属性

        #region 法向量K —— int? NormalK
        /// <summary>
        /// 法向量K
        /// </summary>
        [DependencyProperty]
        public int? NormalK { get; set; }
        #endregion

        #region 特征半径 —— float? FeatureRadius
        /// <summary>
        /// 特征半径
        /// </summary>
        [DependencyProperty]
        public float? FeatureRadius { get; set; }
        #endregion

        #region 角度阈值 —— float? AngleThreshold
        /// <summary>
        /// 角度阈值
        /// </summary>
        [DependencyProperty]
        public float? AngleThreshold { get; set; }
        #endregion

        #region 线程数 —— int? ThreadsCount
        /// <summary>
        /// 线程数
        /// </summary>
        [DependencyProperty]
        public int? ThreadsCount { get; set; }
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
            this.NormalK = 4;
            this.FeatureRadius = 0.05f;
            this.AngleThreshold = 45;
            this.ThreadsCount = 20;

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

            if (!this.NormalK.HasValue)
            {
                MessageBox.Show("法向量K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.FeatureRadius.HasValue)
            {
                MessageBox.Show("特征半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.AngleThreshold.HasValue)
            {
                MessageBox.Show("角度阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ThreadsCount.HasValue)
            {
                MessageBox.Show("线程数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] projectedPoints = await Task.Run(() => this._cloudCommon.ExtractBoundary(points, this.NormalK!.Value, this.FeatureRadius!.Value, this.AngleThreshold!.Value, this.ThreadsCount!.Value));

            IEnumerable<Vector3> positions = projectedPoints.ToVector3s();
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
