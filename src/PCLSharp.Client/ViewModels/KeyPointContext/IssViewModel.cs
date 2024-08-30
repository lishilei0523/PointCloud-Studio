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
using System.Windows.Media;
using Color = System.Windows.Media.Color;

namespace PCLSharp.Client.ViewModels.KeyPointContext
{
    /// <summary>
    /// ISS关键点视图模型
    /// </summary>
    public class IssViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云关键点接口
        /// </summary>
        private readonly ICloudKeyPoints _cloudKeyPoints;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public IssViewModel(ICloudCommon cloudCommon, ICloudKeyPoints cloudKeyPoints)
            : base(cloudCommon)
        {
            this._cloudKeyPoints = cloudKeyPoints;
        }

        #endregion

        #region # 属性

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

        #region 线程数 —— int? ThreadsCount
        /// <summary>
        /// 线程数
        /// </summary>
        [DependencyProperty]
        public int? ThreadsCount { get; set; }
        #endregion

        #region 关键点颜色 —— Color? KeyPointColor
        /// <summary>
        /// 关键点颜色
        /// </summary>
        [DependencyProperty]
        public Color? KeyPointColor { get; set; }
        #endregion

        #region 点云关键点 —— PointGeometry3D KeyPoints
        /// <summary>
        /// 点云关键点
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D KeyPoints { get; set; }
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
            this.SalientRadius = 0.01f;
            this.NonMaxRadius = 0.05f;
            this.Threshold21 = 0.65f;
            this.Threshold32 = 0.1f;
            this.MinNeighborsCount = 4;
            this.ThreadsCount = 10;
            this.KeyPointColor = Colors.Red;

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

            if (!this.SalientRadius.HasValue)
            {
                MessageBox.Show("显著半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.NonMaxRadius.HasValue)
            {
                MessageBox.Show("非极大值抑制半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Threshold21.HasValue)
            {
                MessageBox.Show("二一特征值比上限不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Threshold32.HasValue)
            {
                MessageBox.Show("三二特征值比上限不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinNeighborsCount.HasValue)
            {
                MessageBox.Show("最小邻域点数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ThreadsCount.HasValue)
            {
                MessageBox.Show("线程数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.KeyPointColor.HasValue)
            {
                MessageBox.Show("关键点颜色不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] keyPoints = await Task.Run(() => this._cloudKeyPoints.DetectISS(points, this.SalientRadius!.Value, this.NonMaxRadius!.Value, this.Threshold21!.Value, this.Threshold32!.Value, this.MinNeighborsCount!.Value, this.ThreadsCount!.Value));

            IEnumerable<Vector3> positions = keyPoints.ToVector3s();
            this.KeyPoints = new PointGeometry3D
            {
                Positions = new Vector3Collection(positions)
            };

            this.Idle();
        }
        #endregion

        #region 重置点云 —— override void ResetPointCloud()
        /// <summary>
        /// 重置点云
        /// </summary>
        public override void ResetPointCloud()
        {
            this.KeyPoints = null;
        }
        #endregion

        #endregion
    }
}
