using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;
using System.Windows.Media;

namespace PCLSharp.Client.ViewModels.KeyPointContext
{
    /// <summary>
    /// SIFT关键点视图模型
    /// </summary>
    public class SiftViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public SiftViewModel()
        {
            //默认值
            this.MinScale = 0.05f;
            this.OctavesCount = 6;
            this.ScalesPerOctaveCount = 8;
            this.MinContrast = 0.0005f;
            this.KeyPointColor = Colors.Red;
        }

        #endregion

        #region # 属性

        #region 尺度空间最小标准差 —— float? MinScale
        /// <summary>
        /// 尺度空间最小标准差
        /// </summary>
        [DependencyProperty]
        public float? MinScale { get; set; }
        #endregion

        #region 金字塔组数量 —— int? OctavesCount
        /// <summary>
        /// 金字塔组数量
        /// </summary>
        [DependencyProperty]
        public int? OctavesCount { get; set; }
        #endregion

        #region 每组金字塔计算尺度 —— int? ScalesPerOctaveCount
        /// <summary>
        /// 每组金字塔计算尺度
        /// </summary>
        [DependencyProperty]
        public int? ScalesPerOctaveCount { get; set; }
        #endregion

        #region 限制关键点检测阈值 —— float? MinContrast
        /// <summary>
        /// 限制关键点检测阈值
        /// </summary>
        [DependencyProperty]
        public float? MinContrast { get; set; }
        #endregion

        #region 关键点颜色 —— Color? KeyPointColor
        /// <summary>
        /// 关键点颜色
        /// </summary>
        [DependencyProperty]
        public Color? KeyPointColor { get; set; }
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

            if (!this.MinScale.HasValue)
            {
                MessageBox.Show("尺度空间最小标准偏差不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.OctavesCount.HasValue)
            {
                MessageBox.Show("金字塔组数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ScalesPerOctaveCount.HasValue)
            {
                MessageBox.Show("每组金字塔计算尺度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinContrast.HasValue)
            {
                MessageBox.Show("限制关键点检测阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.KeyPointColor.HasValue)
            {
                MessageBox.Show("关键点颜色不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
