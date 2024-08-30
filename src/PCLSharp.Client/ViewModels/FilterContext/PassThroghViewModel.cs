using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Client.ViewModels.CommonContext;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SharpDX;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace PCLSharp.Client.ViewModels.FilterContext
{
    /// <summary>
    /// 直通滤波视图模型
    /// </summary>
    public class PassThroghViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云滤波接口
        /// </summary>
        private readonly ICloudFilters _cloudFilters;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public PassThroghViewModel(ICloudCommon cloudCommon, ICloudFilters cloudFilters)
            : base(cloudCommon)
        {
            this._cloudFilters = cloudFilters;
        }

        #endregion

        #region # 属性

        #region 已选坐标轴 —— string SelectedAxis
        /// <summary>
        /// 已选坐标轴
        /// </summary>
        [DependencyProperty]
        public string SelectedAxis { get; set; }
        #endregion

        #region 过滤范围最小值 —— float? LimitMin
        /// <summary>
        /// 过滤范围最小值
        /// </summary>
        [DependencyProperty]
        public float? LimitMin { get; set; }
        #endregion

        #region 过滤范围最大值 —— float? LimitMax
        /// <summary>
        /// 过滤范围最大值
        /// </summary>
        [DependencyProperty]
        public float? LimitMax { get; set; }
        #endregion

        #region 坐标轴列表 —— ObservableCollection<string> Axises
        /// <summary>
        /// 坐标轴列表
        /// </summary>
        [DependencyProperty]
        public ObservableCollection<string> Axises { get; set; }
        #endregion

        #endregion

        #region # 方法

        #region 初始化 —— override async Task OnInitializeAsync(CancellationToken cancellationToken)
        /// <summary>
        /// 初始化
        /// </summary>
        protected override Task OnInitializeAsync(CancellationToken cancellationToken)
        {
            //初始化坐标轴
            this.Axises = new ObservableCollection<string>
            {
                Constants.AxisX,
                Constants.AxisY,
                Constants.AxisZ
            };

            //默认值
            this.SelectedAxis = Constants.AxisX;
            this.LimitMin = -1;
            this.LimitMax = 0;

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

            if (string.IsNullOrWhiteSpace(this.SelectedAxis))
            {
                MessageBox.Show("坐标轴不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.LimitMin.HasValue)
            {
                MessageBox.Show("过滤范围最小值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.LimitMax.HasValue)
            {
                MessageBox.Show("过滤范围最大值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.LimitMin.Value >= this.LimitMax.Value)
            {
                MessageBox.Show("过滤范围最大值必须大于最小值！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyPassThrogh(points, this.SelectedAxis, this.LimitMin!.Value, this.LimitMax!.Value));

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
