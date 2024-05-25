using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;
using System.Windows.Media;

namespace PCLSharp.Client.ViewModels.NormalContext
{
    /// <summary>
    /// 基于半径法向量视图模型
    /// </summary>
    public class RadiusBasedViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public RadiusBasedViewModel()
        {
            //默认值
            this.Radius = 0.1f;
            this.NormalLength = 0.02f;
            this.NormalThickness = 0.5f;
            this.NormalColor = Colors.Green;
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

        #endregion

        #region # 方法

        #region 提交 —— async void Submit()
        /// <summary>
        /// 提交
        /// </summary>
        public async void Submit()
        {
            #region # 验证

            if (!this.Radius.HasValue)
            {
                MessageBox.Show("搜索半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
