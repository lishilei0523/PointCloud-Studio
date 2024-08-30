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
    /// SIFT关键点视图模型
    /// </summary>
    public class SiftViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云关键点接口
        /// </summary>
        private readonly ICloudKeyPoints _cloudKeyPoints;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public SiftViewModel(ICloudCommon cloudCommon, ICloudKeyPoints cloudKeyPoints)
            : base(cloudCommon)
        {
            this._cloudKeyPoints = cloudKeyPoints;
        }

        #endregion

        #region # 属性

        #region 尺度空间最小标准差 —— float? MinScale
        /// <summary>
        /// 尺度空间最小标准差
        /// </summary>
        [DependencyProperty]
        public float? MinScale { get; set; }
        #endregion

        #region 金字塔组数量 —— int? OctavesCount
        /// <summary>
        /// 金字塔组数量
        /// </summary>
        [DependencyProperty]
        public int? OctavesCount { get; set; }
        #endregion

        #region 每组金字塔计算尺度 —— int? ScalesPerOctaveCount
        /// <summary>
        /// 每组金字塔计算尺度
        /// </summary>
        [DependencyProperty]
        public int? ScalesPerOctaveCount { get; set; }
        #endregion

        #region 限制关键点检测阈值 —— float? MinContrast
        /// <summary>
        /// 限制关键点检测阈值
        /// </summary>
        [DependencyProperty]
        public float? MinContrast { get; set; }
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
            this.MinScale = 0.05f;
            this.OctavesCount = 6;
            this.ScalesPerOctaveCount = 8;
            this.MinContrast = 0.0005f;
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

            if (!this.MinScale.HasValue)
            {
                MessageBox.Show("尺度空间最小标准偏差不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.OctavesCount.HasValue)
            {
                MessageBox.Show("金字塔组数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ScalesPerOctaveCount.HasValue)
            {
                MessageBox.Show("每组金字塔计算尺度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinContrast.HasValue)
            {
                MessageBox.Show("限制关键点检测阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] keyPoints = await Task.Run(() => this._cloudKeyPoints.DetectSIFT(points, this.MinScale!.Value, this.OctavesCount!.Value, this.ScalesPerOctaveCount!.Value, this.MinContrast!.Value));

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
