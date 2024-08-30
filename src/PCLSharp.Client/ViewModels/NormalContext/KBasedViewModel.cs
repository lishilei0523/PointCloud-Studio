using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Client.ViewModels.CommonContext;
using PCLSharp.Extensions.Helix;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using System.Collections.ObjectModel;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using Color = System.Windows.Media.Color;

namespace PCLSharp.Client.ViewModels.NormalContext
{
    /// <summary>
    /// 基于K法向量视图模型
    /// </summary>
    public class KBasedViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云法向量接口
        /// </summary>
        private readonly ICloudNormals _cloudNormals;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public KBasedViewModel(ICloudCommon cloudCommon, ICloudNormals cloudNormals)
            : base(cloudCommon)
        {
            this._cloudNormals = cloudNormals;
        }

        #endregion

        #region # 属性

        #region 是否启用OMP —— bool OMPEnabled
        /// <summary>
        /// 是否启用OMP
        /// </summary>
        [DependencyProperty]
        public bool OMPEnabled { get; set; }
        #endregion

        #region 搜索近邻数量 —— int? K
        /// <summary>
        /// 搜索近邻数量
        /// </summary>
        [DependencyProperty]
        public int? K { get; set; }
        #endregion

        #region 法向量长度 —— float? NormalLength
        /// <summary>
        /// 法向量长度
        /// </summary>
        [DependencyProperty]
        public float? NormalLength { get; set; }
        #endregion

        #region 法向量厚度 —— float? NormalThickness
        /// <summary>
        /// 法向量厚度
        /// </summary>
        [DependencyProperty]
        public float? NormalThickness { get; set; }
        #endregion

        #region 法向量颜色 —— Color? NormalColor
        /// <summary>
        /// 法向量颜色
        /// </summary>
        [DependencyProperty]
        public Color? NormalColor { get; set; }
        #endregion

        #region 点云法向量 —— ObservableCollection<LineGeometryModel3D> Normals
        /// <summary>
        /// 点云法向量
        /// </summary>
        [DependencyProperty]
        public ObservableCollection<LineGeometryModel3D> Normals { get; set; }
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
            this.OMPEnabled = false;
            this.K = 5;
            this.NormalLength = 0.02f;
            this.NormalThickness = 0.5f;
            this.NormalColor = Colors.Green;
            this.Normals = new ObservableCollection<LineGeometryModel3D>();

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

            if (!this.K.HasValue)
            {
                MessageBox.Show("搜索近邻数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.NormalLength.HasValue)
            {
                MessageBox.Show("法向量长度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.NormalThickness.HasValue)
            {
                MessageBox.Show("法向量厚度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.NormalColor.HasValue)
            {
                MessageBox.Show("法向量颜色不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.PointCloud == null)
            {
                MessageBox.Show("点云不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            this.Busy();

            //清理法向量
            this.Normals.Clear();

            Point3F[] points = this.BasedPointCloud.Points.ToPoint3Fs().ToArray();
            Normal3F[] normals = this.OMPEnabled
                ? await Task.Run(() => this._cloudNormals.EstimateNormalsByKP(points, this.K!.Value))
                : await Task.Run(() => this._cloudNormals.EstimateNormalsByK(points, this.K!.Value));
            for (int i = 0; i < points.Length; i++)
            {
                Point3F point = points[i];
                Normal3F normal = normals[i];

                LineGeometry3D lineGeometry3D = normal.ToLineGeometry3D(point, this.NormalLength!.Value);
                LineGeometryModel3D lineGeometryModel3D = new LineGeometryModel3D();
                lineGeometryModel3D.Geometry = lineGeometry3D;
                lineGeometryModel3D.Color = this.NormalColor!.Value;
                lineGeometryModel3D.Thickness = this.NormalThickness!.Value;
                this.Normals.Add(lineGeometryModel3D);
            }

            this.Idle();
        }
        #endregion

        #region 重置点云 —— override void ResetPointCloud()
        /// <summary>
        /// 重置点云
        /// </summary>
        public override void ResetPointCloud()
        {
            this.Normals.Clear();
        }
        #endregion

        #endregion
    }
}
