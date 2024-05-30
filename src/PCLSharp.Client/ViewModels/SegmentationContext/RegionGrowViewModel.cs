using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.SegmentationContext
{
    /// <summary>
    /// 区域生长分割视图模型
    /// </summary>
    public class RegionGrowViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public RegionGrowViewModel()
        {
            //默认值
            this.NormalK = 50;
            this.ClusterK = 30;
            this.SmoothnessThreshold = 30.0f;
            this.CurvatureThreshold = 1.5f;
            this.MinClusterSize = 1000;
            this.MaxClusterSize = 100000;
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

        #region 簇K —— int? ClusterK
        /// <summary>
        /// 簇K
        /// </summary>
        [DependencyProperty]
        public int? ClusterK { get; set; }
        #endregion

        #region 平滑阈值（角度） —— float? SmoothnessThreshold
        /// <summary>
        /// 平滑阈值（角度）
        /// </summary>
        [DependencyProperty]
        public float? SmoothnessThreshold { get; set; }
        #endregion

        #region 曲率阈值 —— float? CurvatureThreshold
        /// <summary>
        /// 曲率阈值
        /// </summary>
        [DependencyProperty]
        public float? CurvatureThreshold { get; set; }
        #endregion

        #region 簇最小尺寸 —— int? MinClusterSize
        /// <summary>
        /// 簇最小尺寸
        /// </summary>
        [DependencyProperty]
        public int? MinClusterSize { get; set; }
        #endregion

        #region 簇最大尺寸 —— int? MaxClusterSize
        /// <summary>
        /// 簇最大尺寸
        /// </summary>
        [DependencyProperty]
        public int? MaxClusterSize { get; set; }
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
            if (!this.ClusterK.HasValue)
            {
                MessageBox.Show("簇K不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.SmoothnessThreshold.HasValue)
            {
                MessageBox.Show("平滑阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.CurvatureThreshold.HasValue)
            {
                MessageBox.Show("曲率阈值不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MinClusterSize.HasValue)
            {
                MessageBox.Show("簇最小尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.MaxClusterSize.HasValue)
            {
                MessageBox.Show("簇最大尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
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
