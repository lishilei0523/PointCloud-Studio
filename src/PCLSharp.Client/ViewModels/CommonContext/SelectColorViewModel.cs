using Caliburn.Micro;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;
using System.Windows.Media;

namespace PCLSharp.Client.ViewModels.CommonContext
{
    /// <summary>
    /// 选择颜色视图模型
    /// </summary>
    public class SelectColorViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 窗体管理器
        /// </summary>
        private readonly IWindowManager _windowManager;

        /// <summary>
        /// 事件聚合器
        /// </summary>
        private readonly IEventAggregator _eventAggregator;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public SelectColorViewModel(IWindowManager windowManager, IEventAggregator eventAggregator)
        {
            this._windowManager = windowManager;
            this._eventAggregator = eventAggregator;
        }

        #endregion

        #region # 属性

        #region 颜色 —— Color? Color
        /// <summary>
        /// 颜色
        /// </summary>
        [DependencyProperty]
        public Color? Color { get; set; }
        #endregion

        #endregion

        #region # 方法

        //Actions

        #region 提交 —— async void Submit()
        /// <summary>
        /// 提交
        /// </summary>
        public async void Submit()
        {
            #region # 验证

            if (!this.Color.HasValue)
            {
                MessageBox.Show("颜色不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
