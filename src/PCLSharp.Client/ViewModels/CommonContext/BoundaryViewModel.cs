using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.CommonContext
{
    /// <summary>
    /// 提取边界视图模型
    /// </summary>
    public class BoundaryViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public BoundaryViewModel()
        {
            //默认值
            this.NormalK = 4;
            this.FeatureRadius = 0.05f;
            this.AngleThreshold = 45;
            this.ThreadsCount = 20;
        }

        #endregion

        #region # 属性

        #region 法向量K —— int? NormalK
        /// <summary>
        /// 法向量K
        /// </summary>
        [DependencyProperty]
        public int? NormalK { get; set; }
        #endregion

        #region 特征半径 —— float? FeatureRadius
        /// <summary>
        /// 特征半径
        /// </summary>
        [DependencyProperty]
        public float? FeatureRadius { get; set; }
        #endregion

        #region 角度阈值 —— float? AngleThreshold
        /// <summary>
        /// 角度阈值
        /// </summary>
        [DependencyProperty]
        public float? AngleThreshold { get; set; }
        #endregion

        #region 线程数 —— int? ThreadsCount
        /// <summary>
        /// 线程数
        /// </summary>
        [DependencyProperty]
        public int? ThreadsCount { get; set; }
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

            if (!this.NormalK.HasValue)
            {
                MessageBox.Show("法向量K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.FeatureRadius.HasValue)
            {
                MessageBox.Show("特征半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.AngleThreshold.HasValue)
            {
                MessageBox.Show("角度阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.ThreadsCount.HasValue)
            {
                MessageBox.Show("线程数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
