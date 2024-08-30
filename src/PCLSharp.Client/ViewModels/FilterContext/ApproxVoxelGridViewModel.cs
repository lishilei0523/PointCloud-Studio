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
    /// 近似体素降采样视图模型
    /// </summary>
    public class ApproxVoxelGridViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云滤波接口
        /// </summary>
        private readonly ICloudFilters _cloudFilters;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public ApproxVoxelGridViewModel(ICloudCommon cloudCommon, ICloudFilters cloudFilters)
            : base(cloudCommon)
        {
            this._cloudFilters = cloudFilters;
        }

        #endregion

        #region # 属性

        #region 网格尺寸 —— float? LeafSize
        /// <summary>
        /// 网格尺寸
        /// </summary>
        [DependencyProperty]
        public float? LeafSize { get; set; }
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
            this.LeafSize = 0.01f;

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

            if (!this.LeafSize.HasValue)
            {
                MessageBox.Show("网格尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyApproxVoxelGrid(points, this.LeafSize!.Value));

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
