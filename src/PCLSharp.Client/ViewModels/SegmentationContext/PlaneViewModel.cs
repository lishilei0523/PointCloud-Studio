using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Client.ViewModels.CommonContext;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SharpDX;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace PCLSharp.Client.ViewModels.SegmentationContext
{
    /// <summary>
    /// 分割平面视图模型
    /// </summary>
    public class PlaneViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云分割接口
        /// </summary>
        private readonly ICloudSegmentations _cloudSegmentations;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public PlaneViewModel(ICloudCommon cloudCommon, ICloudSegmentations cloudSegmentations)
            : base(cloudCommon)
        {
            this._cloudSegmentations = cloudSegmentations;
        }

        #endregion

        #region # 属性

        #region 是否优化模型系数 —— bool? OptimizeCoefficients
        /// <summary>
        /// 是否优化模型系数
        /// </summary>
        [DependencyProperty]
        public bool? OptimizeCoefficients { get; set; }
        #endregion

        #region 概率 —— float? Probability
        /// <summary>
        /// 概率
        /// </summary>
        [DependencyProperty]
        public float? Probability { get; set; }
        #endregion

        #region 距离阈值 —— float? DistanceThreshold
        /// <summary>
        /// 距离阈值
        /// </summary>
        [DependencyProperty]
        public float? DistanceThreshold { get; set; }
        #endregion

        #region 最大迭代次数 —— int? MaxIterationsCount
        /// <summary>
        /// 最大迭代次数
        /// </summary>
        [DependencyProperty]
        public int? MaxIterationsCount { get; set; }
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
            this.OptimizeCoefficients = true;
            this.Probability = 0.9f;
            this.DistanceThreshold = 0.01f;
            this.MaxIterationsCount = 1000;

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

            if (!this.OptimizeCoefficients.HasValue)
            {
                MessageBox.Show("是否优化模型系数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Probability.HasValue)
            {
                MessageBox.Show("概率不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.DistanceThreshold.HasValue)
            {
                MessageBox.Show("距离阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxIterationsCount.HasValue)
            {
                MessageBox.Show("最大迭代次数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] segmentedPoints = await Task.Run(() => this._cloudSegmentations.SegmentPlane(points, this.OptimizeCoefficients!.Value, this.Probability!.Value, this.DistanceThreshold!.Value, this.MaxIterationsCount!.Value, out int a, out int b, out int c, out int d));

            IEnumerable<Vector3> positions = segmentedPoints.ToVector3s();
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
