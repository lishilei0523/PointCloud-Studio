using HelixToolkit.Wpf.SharpDX;
using PCLSharp.Filters.Interfaces;
using PCLSharp.Primitives.Models;
using Sample.Presentation.Maps;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using SharpDX;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using PerspectiveCamera = HelixToolkit.Wpf.SharpDX.PerspectiveCamera;

namespace Sample.Client.ViewModels.HomeContext
{
    /// <summary>
    /// 首页视图模型
    /// </summary>
    public class IndexViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云滤波接口
        /// </summary>
        private readonly ICloudFilters _cloudFilters;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public IndexViewModel(ICloudFilters cloudFilters)
        {
            this._cloudFilters = cloudFilters;
        }

        #endregion

        #region # 属性

        #region 相机 —— PerspectiveCamera Camera
        /// <summary>
        /// 相机
        /// </summary>
        [DependencyProperty]
        public PerspectiveCamera Camera { get; set; }
        #endregion

        #region 原始点云 —— PointGeometry3D OriginalPointCloud
        /// <summary>
        /// 原始点云
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D OriginalPointCloud { get; set; }
        #endregion

        #region 效果点云 —— PointGeometry3D EffectivePointCloud
        /// <summary>
        /// 效果点云
        /// </summary>
        [DependencyProperty]
        public PointGeometry3D EffectivePointCloud { get; set; }
        #endregion

        #endregion

        #region # 方法

        //Initializations

        #region 初始化 —— Task OnInitializeAsync(CancellationToken cancellationToken)
        /// <summary>
        /// 初始化
        /// </summary>
        protected override async Task OnInitializeAsync(CancellationToken cancellationToken)
        {
            //初始化相机
            this.Camera = new PerspectiveCamera
            {
                LookDirection = new Vector3D(0, 0, -2),
                UpDirection = new Vector3D(0, 1, 0),
                Position = new Point3D(0, 0, 2),
                NearPlaneDistance = 0.125,
                FarPlaneDistance = double.PositiveInfinity,
                FieldOfView = 30
            };

            //加载点云
            await this.ReloadPointCloud();
        }
        #endregion


        //Actions

        #region 适用直通滤波 —— async void ApplyPassThrogh()
        /// <summary>
        /// 适用直通滤波
        /// </summary>
        public async void ApplyPassThrogh()
        {
            this.Busy();

            IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
            ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyPassThrogh(points, "z", 0, 0.05f));

            this.Idle();

            IEnumerable<Vector3> vectors = filterdPoints.Select(x => x.ToVector3());
            this.EffectivePointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(vectors)
            };
        }
        #endregion

        #region 适用均匀降采样 —— async void ApplyUniformSample()
        /// <summary>
        /// 适用均匀降采样
        /// </summary>
        public async void ApplyUniformSample()
        {
            this.Busy();

            IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
            ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyUniformSampling(points, 0.05f));

            this.Idle();

            IEnumerable<Vector3> vectors = filterdPoints.Select(x => x.ToVector3());
            this.EffectivePointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(vectors)
            };
        }
        #endregion

        #region 适用体素降采样 —— async void ApplyVoxelGrid()
        /// <summary>
        /// 适用体素降采样
        /// </summary>
        public async void ApplyVoxelGrid()
        {
            this.Busy();

            IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
            ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyVoxelGrid(points, 0.01f));

            this.Idle();

            IEnumerable<Vector3> vectors = filterdPoints.Select(x => x.ToVector3());
            this.EffectivePointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(vectors)
            };
        }
        #endregion

        #region 适用离群点移除 —— async void ApplyOutlierRemoval()
        /// <summary>
        /// 适用离群点移除
        /// </summary>
        public async void ApplyOutlierRemoval()
        {
            this.Busy();

            IEnumerable<Point3F> points = this.OriginalPointCloud.Points.Select(point => point.ToPoint3F());
            ICollection<Point3F> filterdPoints = await Task.Run(() => this._cloudFilters.ApplyOutlierRemoval(points, 50, 1));

            this.Idle();

            IEnumerable<Vector3> vectors = filterdPoints.Select(x => x.ToVector3());
            this.EffectivePointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(vectors)
            };
        }
        #endregion


        //Private

        #region 加载点云 —— async Task ReloadPointCloud()
        /// <summary>
        /// 加载点云
        /// </summary>
        private async Task ReloadPointCloud()
        {
            //读取ply文件
            string pointCloudPath = "../../../../../assets/table_scene_lms400.ply";
            string[] lines = await Task.Run(() => File.ReadAllLines(pointCloudPath));
            int index = Array.IndexOf(lines, "end_header");

            //解析点集
            IList<Point3F> point3Fs = new List<Point3F>();
            for (int i = index + 1; i < lines.Length; i++)
            {
                string line = lines[i];
                string[] pointsText = line.Split(' ');
                float x = float.Parse(pointsText[0]);
                float y = float.Parse(pointsText[1]);
                float z = float.Parse(pointsText[2]);
                Point3F point3F = new Point3F(x, y, z);
                point3Fs.Add(point3F);
            }

            //几何对象赋值
            Vector3[] pointVectors = point3Fs.Select(point => point.ToVector3()).ToArray();
            this.OriginalPointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(pointVectors)
            };
            this.EffectivePointCloud = new PointGeometry3D
            {
                Positions = new Vector3Collection(pointVectors)
            };
        }
        #endregion

        #endregion
    }
}
