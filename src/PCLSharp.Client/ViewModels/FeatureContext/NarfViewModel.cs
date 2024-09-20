using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.FeatureContext
{
    /// <summary>
    /// NARF特征视图模型
    /// </summary>
    public class NarfViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public NarfViewModel()
        {
            //默认值
            this.AngularResolution = 0.5f;
            this.MaxAngleWidth = 360;
            this.MaxAngleHeight = 180;
            this.NoiseLevel = 0;
            this.MinRange = 0;
            this.BorderSize = 1;
            this.SupportSize = 0.2f;
            this.RotationInvariant = true;
            this.ImageWidth = 1440;
            this.ImageHeight = 870;
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

        #region 旋转不变性 —— bool? RotationInvariant
        /// <summary>
        /// 旋转不变性
        /// </summary>
        [DependencyProperty]
        public bool? RotationInvariant { get; set; }
        #endregion

        #region 直方图宽度 —— int? ImageWidth
        /// <summary>
        /// 直方图宽度
        /// </summary>
        [DependencyProperty]
        public int? ImageWidth { get; set; }
        #endregion

        #region 直方图高度 —— int? ImageHeight
        /// <summary>
        /// 直方图高度
        /// </summary>
        [DependencyProperty]
        public int? ImageHeight { get; set; }
        #endregion

        #endregion

        #region # 方法

        #region 提交 —— async void Submit()
        /// <summary>
        /// 提交
        /// </summary>
        public async void Submit()
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
            if (!this.RotationInvariant.HasValue)
            {
                MessageBox.Show("旋转不变性不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ImageWidth.HasValue)
            {
                MessageBox.Show("直方图宽度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ImageHeight.HasValue)
            {
                MessageBox.Show("直方图高度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
