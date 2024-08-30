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
    /// Harris关键点视图模型
    /// </summary>
    public class HarrisViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云关键点接口
        /// </summary>
        private readonly ICloudKeyPoints _cloudKeyPoints;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public HarrisViewModel(ICloudCommon cloudCommon, ICloudKeyPoints cloudKeyPoints)
            : base(cloudCommon)
        {
            this._cloudKeyPoints = cloudKeyPoints;
        }

        #endregion

        #region # 属性

        #region 非极大值抑制 —— bool? NonMaxSupression
        /// <summary>
        /// 非极大值抑制
        /// </summary>
        [DependencyProperty]
        public bool? NonMaxSupression { get; set; }
        #endregion

        #region 搜索半径 —— float? Radius
        /// <summary>
        /// 搜索半径
        /// </summary>
        [DependencyProperty]
        public float? Radius { get; set; }
        #endregion

        #region 感兴趣阈值 —— float? Threshold
        /// <summary>
        /// 感兴趣阈值
        /// </summary>
        [DependencyProperty]
        public float? Threshold { get; set; }
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
            this.NonMaxSupression = true;
            this.Radius = 0.008f;
            this.Threshold = 0.0005f;
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

            if (!this.NonMaxSupression.HasValue)
            {
                MessageBox.Show("非极大值抑制不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Radius.HasValue)
            {
                MessageBox.Show("搜索半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Threshold.HasValue)
            {
                MessageBox.Show("感兴趣阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] keyPoints = await Task.Run(() => this._cloudKeyPoints.DetectHarris(points, this.NonMaxSupression!.Value, this.Radius!.Value, this.Threshold!.Value));

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
