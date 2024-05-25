using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.FilterContext
{
    /// <summary>
    /// 均匀采样视图模型
    /// </summary>
    public class UniformSampleViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public UniformSampleViewModel()
        {
            //默认值
            this.Radius = 0.05f;
        }

        #endregion

        #region # 属性

        #region 采样半径 —— float? Radius
        /// <summary>
        /// 采样半径
        /// </summary>
        [DependencyProperty]
        public float? Radius { get; set; }
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
                MessageBox.Show("采样半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
