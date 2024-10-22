﻿using HelixToolkit.Wpf.SharpDX;
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
using System.Windows.Media;
using Color = System.Windows.Media.Color;

namespace PCLSharp.Client.ViewModels.KeyPointContext
{
    /// <summary>
    /// NARF关键点视图模型
    /// </summary>
    public class NarfViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 点云关键点接口
        /// </summary>
        private readonly ICloudKeyPoints _cloudKeyPoints;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public NarfViewModel(ICloudCommon cloudCommon, ICloudKeyPoints cloudKeyPoints)
            : base(cloudCommon)
        {
            this._cloudKeyPoints = cloudKeyPoints;
        }

        #endregion

        #region # 属性

        #region 角度分辨率 —— float? AngularResolution
        /// <summary>
        /// 角度分辨率
        /// </summary>
        [DependencyProperty]
        public float? AngularResolution { get; set; }
        #endregion

        #region 水平边界角度 —— float? MaxAngleWidth
        /// <summary>
        /// 水平边界角度
        /// </summary>
        [DependencyProperty]
        public float? MaxAngleWidth { get; set; }
        #endregion

        #region 垂直边界角度 —— float? MaxAngleHeight
        /// <summary>
        /// 垂直边界角度
        /// </summary>
        [DependencyProperty]
        public float? MaxAngleHeight { get; set; }
        #endregion

        #region 最近点最大距离 —— float? NoiseLevel
        /// <summary>
        /// 最近点最大距离
        /// </summary>
        [DependencyProperty]
        public float? NoiseLevel { get; set; }
        #endregion

        #region 最小可见范围 —— float? MinRange
        /// <summary>
        /// 最小可见范围
        /// </summary>
        [DependencyProperty]
        public float? MinRange { get; set; }
        #endregion

        #region 边界尺寸 —— int? BorderSize
        /// <summary>
        /// 边界尺寸
        /// </summary>
        [DependencyProperty]
        public int? BorderSize { get; set; }
        #endregion

        #region 计算范围半径 —— float? SupportSize
        /// <summary>
        /// 计算范围半径
        /// </summary>
        [DependencyProperty]
        public float? SupportSize { get; set; }
        #endregion

        #region 关键点颜色 —— Color? KeyPointColor
        /// <summary>
        /// 关键点颜色
        /// </summary>
        [DependencyProperty]
        public Color? KeyPointColor { get; set; }
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
            this.AngularResolution = 0.5f;
            this.MaxAngleWidth = 360;
            this.MaxAngleHeight = 180;
            this.NoiseLevel = 0;
            this.MinRange = 0;
            this.BorderSize = 1;
            this.SupportSize = 0.2f;
            this.KeyPointColor = Colors.Red;

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

            if (!this.AngularResolution.HasValue)
            {
                MessageBox.Show("角度分辨率不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxAngleWidth.HasValue)
            {
                MessageBox.Show("水平边界角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxAngleHeight.HasValue)
            {
                MessageBox.Show("垂直边界角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.NoiseLevel.HasValue)
            {
                MessageBox.Show("最近点最大距离不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinRange.HasValue)
            {
                MessageBox.Show("最小可见范围不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.BorderSize.HasValue)
            {
                MessageBox.Show("边界尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.SupportSize.HasValue)
            {
                MessageBox.Show("计算范围半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.KeyPointColor.HasValue)
            {
                MessageBox.Show("关键点颜色不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
            Point3F[] keyPoints = await Task.Run(() => this._cloudKeyPoints.DetectNARF(points, this.AngularResolution!.Value, this.MaxAngleWidth!.Value, this.MaxAngleHeight!.Value, this.NoiseLevel!.Value, this.MinRange!.Value, this.BorderSize!.Value, this.SupportSize!.Value));

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

        #endregion
    }
}
