using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.SearchContext
{
    /// <summary>
    /// 八叉树搜索视图模型
    /// </summary>
    public class OctreeSearchViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public OctreeSearchViewModel()
        {
            //默认值
            this.Resolution = 128.0f;
            this.ReferencePointX = 0.0f;
            this.ReferencePointY = 0.0f;
            this.ReferencePointZ = 0.0f;
        }

        #endregion

        #region # 属性

        #region 分辨率 —— float? Resolution
        /// <summary>
        /// 分辨率
        /// </summary>
        [DependencyProperty]
        public float? Resolution { get; set; }
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

            if (!this.Resolution.HasValue)
            {
                MessageBox.Show("分辨率不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
