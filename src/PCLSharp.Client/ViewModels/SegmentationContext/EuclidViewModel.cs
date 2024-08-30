using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Client.ViewModels.CommonContext;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Extensions;
using SharpDX;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using Color = System.Windows.Media.Color;

namespace PCLSharp.Client.ViewModels.SegmentationContext
{
    /// <summary>
    /// 欧氏聚类分割视图模型
    /// </summary>
    public class EuclidViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云分割接口
        /// </summary>
        private readonly ICloudSegmentations _cloudSegmentations;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public EuclidViewModel(ICloudCommon cloudCommon, ICloudSegmentations cloudSegmentations)
            : base(cloudCommon)
        {
            this._cloudSegmentations = cloudSegmentations;
        }

        #endregion

        #region # 属性

        #region 簇搜索容差 —— float? ClusterTolerance
        /// <summary>
        /// 簇搜索容差
        /// </summary>
        [DependencyProperty]
        public float? ClusterTolerance { get; set; }
        #endregion

        #region 簇最小尺寸 —— int? MinClusterSize
        /// <summary>
        /// 簇最小尺寸
        /// </summary>
        [DependencyProperty]
        public int? MinClusterSize { get; set; }
        #endregion

        #region 簇最大尺寸 —— int? MaxClusterSize
        /// <summary>
        /// 簇最大尺寸
        /// </summary>
        [DependencyProperty]
        public int? MaxClusterSize { get; set; }
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
            this.ClusterTolerance = 1.5f;
            this.MinClusterSize = 1000;
            this.MaxClusterSize = 100000;

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

            if (!this.ClusterTolerance.HasValue)
            {
                MessageBox.Show("簇搜索容差不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinClusterSize.HasValue)
            {
                MessageBox.Show("簇最小尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxClusterSize.HasValue)
            {
                MessageBox.Show("簇最大尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[][] pointsClusters = await Task.Run(() => this._cloudSegmentations.EuclidClusterSegment(points, this.ClusterTolerance!.Value, this.MinClusterSize!.Value, this.MaxClusterSize!.Value));

            Vector3Collection positions = new Vector3Collection();
            Color4Collection colors = new Color4Collection();
            for (int clusterIndex = 0; clusterIndex < pointsClusters.Length; clusterIndex++)
            {
                Point3F[] pointsCluster = pointsClusters[clusterIndex];
                Color color = ColorExtension.RandomColor();
                if (clusterIndex % 2 == 0)
                {
                    color = color.Invert();
                }
                IEnumerable<Vector3> clusterPositions = pointsCluster.ToVector3s();
                IEnumerable<Color4> clusterColors = pointsCluster.Select(x => color.ToColor4());
                positions.AddRange(clusterPositions);
                colors.AddRange(clusterColors);
            }
            this.PointCloud = new PointGeometry3D
            {
                Positions = positions,
                Colors = colors
            };

            this.Idle();
        }
        #endregion

        #endregion
    }
}
