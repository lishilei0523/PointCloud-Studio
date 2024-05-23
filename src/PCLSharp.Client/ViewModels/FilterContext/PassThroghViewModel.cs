using Caliburn.Micro;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Collections.ObjectModel;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace PCLSharp.Client.ViewModels.FilterContext
{
    /// <summary>
    /// 直通滤波视图模型
    /// </summary>
    public class PassThroghViewModel : ScreenBase
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
        public PassThroghViewModel(IWindowManager windowManager, IEventAggregator eventAggregator)
        {
            this._windowManager = windowManager;
            this._eventAggregator = eventAggregator;
        }

        #endregion

        #region # 属性

        #region 已选坐标轴 —— string SelectedAxis
        /// <summary>
        /// 已选坐标轴
        /// </summary>
        [DependencyProperty]
        public string SelectedAxis { get; set; }
        #endregion

        #region 过滤范围最小值 —— float? LimitMin
        /// <summary>
        /// 过滤范围最小值
        /// </summary>
        [DependencyProperty]
        public float? LimitMin { get; set; }
        #endregion

        #region 过滤范围最大值 —— float? LimitMax
        /// <summary>
        /// 过滤范围最大值
        /// </summary>
        [DependencyProperty]
        public float? LimitMax { get; set; }
        #endregion

        #region 坐标轴列表 —— ObservableCollection<string> Axises
        /// <summary>
        /// 坐标轴列表
        /// </summary>
        [DependencyProperty]
        public ObservableCollection<string> Axises { get; set; }
        #endregion

        #endregion

        #region # 方法

        //Initializations

        #region 初始化 —— override async Task OnInitializeAsync(CancellationToken cancellationToken)
        /// <summary>
        /// 初始化
        /// </summary>
        protected override Task OnInitializeAsync(CancellationToken cancellationToken)
        {
            //初始化坐标轴
            this.Axises = new ObservableCollection<string>
            {
                Constants.AxisX,
                Constants.AxisY,
                Constants.AxisZ
            };

            return base.OnInitializeAsync(cancellationToken);
        }
        #endregion


        //Actions

        #region 提交 —— async void Submit()
        /// <summary>
        /// 提交
        /// </summary>
        public async void Submit()
        {
            #region # 验证

            if (string.IsNullOrWhiteSpace(this.SelectedAxis))
            {
                MessageBox.Show("坐标轴不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.LimitMin.HasValue)
            {
                MessageBox.Show("过滤范围最小值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.LimitMax.HasValue)
            {
                MessageBox.Show("过滤范围最大值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (this.LimitMin.Value >= this.LimitMax.Value)
            {
                MessageBox.Show("过滤范围最大值必须大于最小值！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
