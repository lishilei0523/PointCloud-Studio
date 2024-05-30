using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.CommonContext
{
    /// <summary>
    /// 投射平面视图模型
    /// </summary>
    public class ProjectViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public ProjectViewModel()
        {
            //默认值
            this.A = 1;
            this.B = 0;
            this.C = 0;
            this.D = 0;
        }

        #endregion

        #region # 属性

        #region 平面方程系数a —— float? A
        /// <summary>
        /// 平面方程系数a
        /// </summary>
        [DependencyProperty]
        public float? A { get; set; }
        #endregion

        #region 平面方程系数b —— float? B
        /// <summary>
        /// 平面方程系数b
        /// </summary>
        [DependencyProperty]
        public float? B { get; set; }
        #endregion

        #region 平面方程系数c —— float? C
        /// <summary>
        /// 平面方程系数c
        /// </summary>
        [DependencyProperty]
        public float? C { get; set; }
        #endregion

        #region 平面方程系数d —— float? D
        /// <summary>
        /// 平面方程系数d
        /// </summary>
        [DependencyProperty]
        public float? D { get; set; }
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

            if (!this.A.HasValue)
            {
                MessageBox.Show("平面方程系数a不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.B.HasValue)
            {
                MessageBox.Show("平面方程系数b不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.C.HasValue)
            {
                MessageBox.Show("平面方程系数c不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.D.HasValue)
            {
                MessageBox.Show("平面方程系数d不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
