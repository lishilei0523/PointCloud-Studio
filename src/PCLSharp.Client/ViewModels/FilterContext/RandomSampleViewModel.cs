using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Client.ViewModels.CommonContext;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace PCLSharp.Client.ViewModels.FilterContext
{
    /// <summary>
    /// 随机采样视图模型
    /// </summary>
    public class RandomSampleViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云滤波接口
        /// </summary>
        private readonly ICloudFilters _cloudFilters;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public RandomSampleViewModel(ICloudCommon cloudCommon, ICloudFilters cloudFilters)
            : base(cloudCommon)
        {
            this._cloudFilters = cloudFilters;
        }

        #endregion

        #region # 属性

        #region 随机种子 —— int? Seed
        /// <summary>
        /// 随机种子
        /// </summary>
        [DependencyProperty]
        public int? Seed { get; set; }
        #endregion

        #region 采样数量 —— int? SamplesCount
        /// <summary>
        /// 采样数量
        /// </summary>
        [DependencyProperty]
        public int? SamplesCount { get; set; }
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
            this.Seed = (int)Math.Abs(int.MaxValue - DateTime.Now.Ticks);
            this.SamplesCount = 2000;

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

            if (!this.Seed.HasValue)
            {
                MessageBox.Show("随机种子不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.SamplesCount.HasValue)
            {
                MessageBox.Show("采样数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] filterdPoints = await Task.Run(() => this._cloudFilters.ApplyRandomSampling(points, this.Seed!.Value, this.SamplesCount!.Value));

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
