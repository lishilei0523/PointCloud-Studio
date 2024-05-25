using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;
using System.Windows.Media;

namespace PCLSharp.Client.ViewModels.KeyPointContext
{
    /// <summary>
    /// ISS关键点视图模型
    /// </summary>
    public class IssViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public IssViewModel()
        {
            //默认值
            this.SalientRadius = 0.01f;
            this.NonMaxRadius = 0.05f;
            this.Threshold21 = 0.65f;
            this.Threshold32 = 0.1f;
            this.MinNeighborsCount = 4;
            this.ThreadsCount = 10;
            this.KeyPointColor = Colors.Red;
        }

        #endregion

        #region # 属性

        #region 显著半径 —— float? SalientRadius
        /// <summary>
        /// 显著半径
        /// </summary>
        [DependencyProperty]
        public float? SalientRadius { get; set; }
        #endregion

        #region 抑制半径 —— float? NonMaxRadius
        /// <summary>
        /// 抑制半径
        /// </summary>
        [DependencyProperty]
        public float? NonMaxRadius { get; set; }
        #endregion

        #region 二一比上限 —— float? Threshold21
        /// <summary>
        /// 二一比上限
        /// </summary>
        [DependencyProperty]
        public float? Threshold21 { get; set; }
        #endregion

        #region 三二比上限 —— float? Threshold32
        /// <summary>
        /// 三二比上限
        /// </summary>
        [DependencyProperty]
        public float? Threshold32 { get; set; }
        #endregion

        #region 最小邻域点数 —— int? MinNeighborsCount
        /// <summary>
        /// 最小邻域点数
        /// </summary>
        [DependencyProperty]
        public int? MinNeighborsCount { get; set; }
        #endregion

        #region 线程数 —— int? ThreadsCount
        /// <summary>
        /// 线程数
        /// </summary>
        [DependencyProperty]
        public int? ThreadsCount { get; set; }
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

            if (!this.SalientRadius.HasValue)
            {
                MessageBox.Show("显著半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.NonMaxRadius.HasValue)
            {
                MessageBox.Show("非极大值抑制半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Threshold21.HasValue)
            {
                MessageBox.Show("二一特征值比上限不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Threshold32.HasValue)
            {
                MessageBox.Show("三二特征值比上限不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinNeighborsCount.HasValue)
            {
                MessageBox.Show("最小邻域点数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ThreadsCount.HasValue)
            {
                MessageBox.Show("线程数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
