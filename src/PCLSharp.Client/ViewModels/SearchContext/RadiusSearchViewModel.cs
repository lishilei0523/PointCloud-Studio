using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.SearchContext
{
    /// <summary>
    /// 半径搜索视图模型
    /// </summary>
    public class RadiusSearchViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public RadiusSearchViewModel()
        {
            //默认值
            this.Radius = 0.5f;
            this.ReferencePointX = 0.0f;
            this.ReferencePointY = 0.0f;
            this.ReferencePointZ = 0.0f;
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

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
