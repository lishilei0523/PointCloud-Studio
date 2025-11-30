using HelixToolkit;
using HelixToolkit.SharpDX;
using PCLSharp.Client.ViewModels.CommonContext;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace PCLSharp.Client.ViewModels.SurfaceContext
{
    /// <summary>
    /// 贪婪三角化视图模型
    /// </summary>
    public class GreedyProjectionViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云分割接口
        /// </summary>
        private readonly ICloudSurfaces _cloudSurfaces;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public GreedyProjectionViewModel(ICloudCommon cloudCommon, ICloudSurfaces cloudSurfaces)
            : base(cloudCommon)
        {
            this._cloudSurfaces = cloudSurfaces;
        }

        #endregion

        #region # 属性

        #region 网格几何 —— MeshGeometry3D MeshGeometry3D
        /// <summary>
        /// 网格几何
        /// </summary>
        [DependencyProperty]
        public MeshGeometry3D MeshGeometry3D { get; set; }
        #endregion

        #region 法向量K —— int? NormalK
        /// <summary>
        /// 法向量K
        /// </summary>
        [DependencyProperty]
        public int? NormalK { get; set; }
        #endregion

        #region 搜索半径 —— float? SearchRadius
        /// <summary>
        /// 搜索半径
        /// </summary>
        [DependencyProperty]
        public float? SearchRadius { get; set; }
        #endregion

        #region 近邻点最远倍数 —— float? Mu
        /// <summary>
        /// 近邻点最远倍数
        /// </summary>
        [DependencyProperty]
        public float? Mu { get; set; }
        #endregion

        #region 最大邻域数 —— int? MaxNearestNeighbors
        /// <summary>
        /// 最大邻域数
        /// </summary>
        [DependencyProperty]
        public int? MaxNearestNeighbors { get; set; }
        #endregion

        #region 偏离法向量最大角度 —— double? MaxSurfaceAngle
        /// <summary>
        /// 曲率阈值
        /// </summary>
        [DependencyProperty]
        public double? MaxSurfaceAngle { get; set; }
        #endregion

        #region 三角形最小角度 —— double? MinAngle
        /// <summary>
        /// 三角形最小角度
        /// </summary>
        [DependencyProperty]
        public double? MinAngle { get; set; }
        #endregion

        #region 三角形最大角度 —— double? MaxAngle
        /// <summary>
        /// 三角形最大角度
        /// </summary>
        [DependencyProperty]
        public double? MaxAngle { get; set; }
        #endregion

        #region 保证法向量朝向一致 —— bool NormalConsistency
        /// <summary>
        /// 保证法向量朝向一致
        /// </summary>
        [DependencyProperty]
        public bool NormalConsistency { get; set; }
        #endregion

        #region 线程数 —— int? ThreadsCount
        /// <summary>
        /// 线程数
        /// </summary>
        [DependencyProperty]
        public int? ThreadsCount { get; set; }
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
            this.NormalK = 50;
            this.SearchRadius = 0.025f;
            this.Mu = 2.5f;
            this.MaxNearestNeighbors = 100;
            this.MaxSurfaceAngle = 45;
            this.MinAngle = 10;
            this.MaxAngle = 120;
            this.NormalConsistency = false;
            this.ThreadsCount = 20;

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

            if (!this.NormalK.HasValue)
            {
                MessageBox.Show("法向量K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.SearchRadius.HasValue)
            {
                MessageBox.Show("搜索半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Mu.HasValue)
            {
                MessageBox.Show("近邻点最远倍数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxNearestNeighbors.HasValue)
            {
                MessageBox.Show("最大邻域数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxSurfaceAngle.HasValue)
            {
                MessageBox.Show("偏离法向量最大角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinAngle.HasValue)
            {
                MessageBox.Show("三角形最小角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxAngle.HasValue)
            {
                MessageBox.Show("三角形最大角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ThreadsCount.HasValue)
            {
                MessageBox.Show("线程数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            MeshGeometryInfo meshGeometryInfo = await Task.Run(() => this._cloudSurfaces.ApplyGreedyProjection(points, this.NormalK!.Value, this.SearchRadius!.Value, this.Mu!.Value, this.MaxNearestNeighbors!.Value, this.MaxSurfaceAngle!.Value, this.MinAngle!.Value, this.MaxAngle!.Value, this.NormalConsistency, this.ThreadsCount!.Value));

            IEnumerable<Vector3> positions = meshGeometryInfo.Positions.ToVector3s();
            this.MeshGeometry3D = new MeshGeometry3D
            {
                Positions = new Vector3Collection(positions),
                Indices = new IntCollection(meshGeometryInfo.TriangleIndices)
            };

            this.Idle();
        }
        #endregion

        #endregion
    }
}
