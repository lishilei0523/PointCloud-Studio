using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.FilterContext
{
    /// <summary>
    /// 统计离群点移除视图模型
    /// </summary>
    public class StatOutlierRemovalViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public StatOutlierRemovalViewModel()
        {

        }

        #endregion

        #region # 属性

        #region 平均近邻K —— int? MeanK
        /// <summary>
        /// 平均近邻K
        /// </summary>
        [DependencyProperty]
        public int? MeanK { get; set; }
        #endregion

        #region 标准差系数 —— int? StddevMult
        /// <summary>
        /// 标准差系数
        /// </summary>
        [DependencyProperty]
        public float? StddevMult { get; set; }
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

            if (!this.MeanK.HasValue)
            {
                MessageBox.Show("平均距离估计的最近邻居的数量不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.StddevMult.HasValue)
            {
                MessageBox.Show("标准差阈值系数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
