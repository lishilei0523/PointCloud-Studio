using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;
using System.Windows.Media;

namespace PCLSharp.Client.ViewModels.KeyPointContext
{
    /// <summary>
    /// Harris关键点视图模型
    /// </summary>
    public class HarrisViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public HarrisViewModel()
        {
            //默认值
            this.NonMaxSupression = true;
            this.Radius = 0.008f;
            this.Threshold = 0.0005f;
            this.KeyPointColor = Colors.Red;
        }

        #endregion

        #region # 属性

        #region 非极大值抑制 —— bool? NonMaxSupression
        /// <summary>
        /// 非极大值抑制
        /// </summary>
        [DependencyProperty]
        public bool? NonMaxSupression { get; set; }
        #endregion

        #region 搜索半径 —— float? Radius
        /// <summary>
        /// 搜索半径
        /// </summary>
        [DependencyProperty]
        public float? Radius { get; set; }
        #endregion

        #region 感兴趣阈值 —— float? Threshold
        /// <summary>
        /// 感兴趣阈值
        /// </summary>
        [DependencyProperty]
        public float? Threshold { get; set; }
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

            if (!this.NonMaxSupression.HasValue)
            {
                MessageBox.Show("非极大值抑制不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Radius.HasValue)
            {
                MessageBox.Show("搜索半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Threshold.HasValue)
            {
                MessageBox.Show("感兴趣阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
