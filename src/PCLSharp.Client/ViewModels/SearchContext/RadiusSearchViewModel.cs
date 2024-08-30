using HelixToolkit.Wpf.SharpDX;
using HelixToolkit.Wpf.SharpDX.Model.Scene;
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
using System.Windows.Input;
using System.Windows.Media.Media3D;
using Point = System.Windows.Point;

namespace PCLSharp.Client.ViewModels.SearchContext
{
    /// <summary>
    /// 半径搜索视图模型
    /// </summary>
    public class RadiusSearchViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云搜索接口
        /// </summary>
        private readonly ICloudSearch _cloudSearch;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public RadiusSearchViewModel(ICloudCommon cloudCommon, ICloudSearch cloudSearch)
            : base(cloudCommon)
        {
            this._cloudSearch = cloudSearch;
        }

        #endregion

        #region # 属性

        #region 搜索半径 —— float? Radius
        /// <summary>
        /// 搜索半径
        /// </summary>
        [DependencyProperty]
        public float? Radius { get; set; }
        #endregion

        #region 参考点X —— float? ReferencePointX
        /// <summary>
        /// 参考点X
        /// </summary>
        [DependencyProperty]
        public float? ReferencePointX { get; set; }
        #endregion

        #region 参考点Y —— float? ReferencePointY
        /// <summary>
        /// 参考点Y
        /// </summary>
        [DependencyProperty]
        public float? ReferencePointY { get; set; }
        #endregion

        #region 参考点Z —— float? ReferencePointZ
        /// <summary>
        /// 参考点Z
        /// </summary>
        [DependencyProperty]
        public float? ReferencePointZ { get; set; }
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
            this.Radius = 0.5f;
            this.ReferencePointX = 0.0f;
            this.ReferencePointY = 0.0f;
            this.ReferencePointZ = 0.0f;

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

            if (!this.Radius.HasValue)
            {
                MessageBox.Show("搜索半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ReferencePointX.HasValue)
            {
                MessageBox.Show("参考点X不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ReferencePointY.HasValue)
            {
                MessageBox.Show("参考点Y不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ReferencePointZ.HasValue)
            {
                MessageBox.Show("参考点Z不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F referencePoint = new Point3F(this.ReferencePointX!.Value, this.ReferencePointY!.Value, this.ReferencePointZ!.Value);
            Point3F[] keyPoints = await Task.Run(() => this._cloudSearch.RadiusSearch(points, referencePoint, this.Radius!.Value));

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

        #region 鼠标左键按下事件 —— void ViewportOnMouseLeftDown(Viewport3DX viewport3D...
        /// <summary>
        /// 鼠标左键按下事件
        /// </summary>
        public void ViewportOnMouseLeftDown(Viewport3DX viewport3D, MouseButtonEventArgs eventArgs)
        {
            Point mousePos2D = eventArgs.GetPosition(viewport3D);
            bool success = viewport3D.FindNearest(mousePos2D, out Point3D mousePos3D, out Vector3D _, out Element3D _, out SceneNode _);

            //获得焦点
            if (success)
            {
                viewport3D.LookAt(mousePos3D, 200);
                this.ReferencePointX = (float)mousePos3D.X;
                this.ReferencePointY = (float)mousePos3D.Y;
                this.ReferencePointZ = (float)mousePos3D.Z;

                eventArgs.Handled = true;
            }
        }
        #endregion

        #endregion
    }
}
