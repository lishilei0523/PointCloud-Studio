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
    /// 投射平面视图模型
    /// </summary>
    public class ProjectViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云通用操作接口;
        /// </summary>
        private readonly ICloudCommon _cloudCommon;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public ProjectViewModel(ICloudCommon cloudCommon)
            : base(cloudCommon)
        {
            this._cloudCommon = cloudCommon;
        }

        #endregion

        #region # 属性

        #region 平面方程系数a —— float? A
        /// <summary>
        /// 平面方程系数a
        /// </summary>
        [DependencyProperty]
        public float? A { get; set; }
        #endregion

        #region 平面方程系数b —— float? B
        /// <summary>
        /// 平面方程系数b
        /// </summary>
        [DependencyProperty]
        public float? B { get; set; }
        #endregion

        #region 平面方程系数c —— float? C
        /// <summary>
        /// 平面方程系数c
        /// </summary>
        [DependencyProperty]
        public float? C { get; set; }
        #endregion

        #region 平面方程系数d —— float? D
        /// <summary>
        /// 平面方程系数d
        /// </summary>
        [DependencyProperty]
        public float? D { get; set; }
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
            this.A = 1;
            this.B = 0;
            this.C = 0;
            this.D = 0;

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

            if (!this.A.HasValue)
            {
                MessageBox.Show("平面方程系数a不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.B.HasValue)
            {
                MessageBox.Show("平面方程系数b不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.C.HasValue)
            {
                MessageBox.Show("平面方程系数c不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.D.HasValue)
            {
                MessageBox.Show("平面方程系数d不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] projectedPoints = await Task.Run(() => this._cloudCommon.ProjectPlane(points, this.A!.Value, this.B!.Value, this.C!.Value, this.D!.Value));

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
