using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.FilterContext
{
    /// <summary>
    /// 体素降采样视图模型
    /// </summary>
    public class VoxelGridViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public VoxelGridViewModel()
        {
            //默认值
            this.LeafSize = 0.01f;
        }

        #endregion

        #region # 属性

        #region 网格尺寸 —— float? LeafSize
        /// <summary>
        /// 网格尺寸
        /// </summary>
        [DependencyProperty]
        public float? LeafSize { get; set; }
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

            if (!this.LeafSize.HasValue)
            {
                MessageBox.Show("网格尺寸不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
