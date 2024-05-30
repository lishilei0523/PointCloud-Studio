using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.SegmentationContext
{
    /// <summary>
    /// 分割球体视图模型
    /// </summary>
    public class SphereViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public SphereViewModel()
        {
            //默认值
            this.OptimizeCoefficients = true;
            this.Probability = 0.9f;
            this.DistanceThreshold = 0.01f;
            this.MinRadius = 0;
            this.MaxRadius = 10.0f;
            this.MaxIterationsCount = 1000;
        }

        #endregion

        #region # 属性

        #region 是否优化模型系数 —— bool? OptimizeCoefficients
        /// <summary>
        /// 是否优化模型系数
        /// </summary>
        [DependencyProperty]
        public bool? OptimizeCoefficients { get; set; }
        #endregion

        #region 概率 —— float? Probability
        /// <summary>
        /// 概率
        /// </summary>
        [DependencyProperty]
        public float? Probability { get; set; }
        #endregion

        #region 距离阈值 —— float? DistanceThreshold
        /// <summary>
        /// 距离阈值
        /// </summary>
        [DependencyProperty]
        public float? DistanceThreshold { get; set; }
        #endregion

        #region 最小半径 —— float? MinRadius
        /// <summary>
        /// 最小半径
        /// </summary>
        [DependencyProperty]
        public float? MinRadius { get; set; }
        #endregion

        #region 最大半径 —— float? MaxRadius
        /// <summary>
        /// 最大半径
        /// </summary>
        [DependencyProperty]
        public float? MaxRadius { get; set; }
        #endregion

        #region 最大迭代次数 —— int? MaxIterationsCount
        /// <summary>
        /// 最大迭代次数
        /// </summary>
        [DependencyProperty]
        public int? MaxIterationsCount { get; set; }
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

            if (!this.OptimizeCoefficients.HasValue)
            {
                MessageBox.Show("是否优化模型系数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Probability.HasValue)
            {
                MessageBox.Show("概率不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.DistanceThreshold.HasValue)
            {
                MessageBox.Show("距离阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinRadius.HasValue)
            {
                MessageBox.Show("最小半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxRadius.HasValue)
            {
                MessageBox.Show("最大半径不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxIterationsCount.HasValue)
            {
                MessageBox.Show("最大迭代次数不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
