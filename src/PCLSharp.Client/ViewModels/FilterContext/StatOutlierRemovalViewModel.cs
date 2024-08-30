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

namespace PCLSharp.Client.ViewModels.FilterContext
{
    /// <summary>
    /// 统计离群点移除视图模型
    /// </summary>
    public class StatOutlierRemovalViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云滤波接口
        /// </summary>
        private readonly ICloudFilters _cloudFilters;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public StatOutlierRemovalViewModel(ICloudCommon cloudCommon, ICloudFilters cloudFilters)
            : base(cloudCommon)
        {
            this._cloudFilters = cloudFilters;
        }

        #endregion

        #region # 属性

        #region 平均近邻K —— int? MeanK
        /// <summary>
        /// 平均近邻K
        /// </summary>
        [DependencyProperty]
        public int? MeanK { get; set; }
        #endregion

        #region 标准差系数 —— float? StddevMult
        /// <summary>
        /// 标准差系数
        /// </summary>
        [DependencyProperty]
        public float? StddevMult { get; set; }
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
            this.MeanK = 50;
            this.StddevMult = 1.0f;

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

            if (!this.MeanK.HasValue)
            {
                MessageBox.Show("平均距离估计的最近邻居的数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.StddevMult.HasValue)
            {
                MessageBox.Show("标准差阈值系数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyStatisticalOutlierRemoval(points, this.MeanK!.Value, this.StddevMult!.Value));

            IEnumerable<Vector3> positions = filterdPoints.ToVector3s();
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
