using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows;

namespace PCLSharp.Client.ViewModels.CommonContext
{
    /// <summary>
    /// 仿射变换视图模型
    /// </summary>
    public class AffineViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public AffineViewModel()
        {
            //默认值
            this.X = 0.1f;
            this.Y = 0.1f;
            this.Z = 0.1f;
            this.RX = 15f;
            this.RY = 15;
            this.RZ = 0;
        }

        #endregion

        #region # 属性

        #region X轴位置 —— float? X
        /// <summary>
        /// X轴位置
        /// </summary>
        [DependencyProperty]
        public float? X { get; set; }
        #endregion

        #region Y轴位置 —— float? Y
        /// <summary>
        /// Y轴位置
        /// </summary>
        [DependencyProperty]
        public float? Y { get; set; }
        #endregion

        #region Z轴位置 —— float? Z
        /// <summary>
        /// Z轴位置
        /// </summary>
        [DependencyProperty]
        public float? Z { get; set; }
        #endregion

        #region X轴旋转角度 —— float? RX
        /// <summary>
        /// X轴旋转角度
        /// </summary>
        [DependencyProperty]
        public float? RX { get; set; }
        #endregion

        #region Y轴旋转角度 —— float? RY
        /// <summary>
        /// Y轴旋转角度
        /// </summary>
        [DependencyProperty]
        public float? RY { get; set; }
        #endregion

        #region Z轴旋转角度 —— float? RZ
        /// <summary>
        /// Z轴旋转角度
        /// </summary>
        [DependencyProperty]
        public float? RZ { get; set; }
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

            if (!this.X.HasValue)
            {
                MessageBox.Show("X轴位置不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Y.HasValue)
            {
                MessageBox.Show("Y轴位置不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.Z.HasValue)
            {
                MessageBox.Show("Z轴位置不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.RX.HasValue)
            {
                MessageBox.Show("X轴旋转角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.RY.HasValue)
            {
                MessageBox.Show("Y轴旋转角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            if (!this.RZ.HasValue)
            {
                MessageBox.Show("Z轴旋转角度不可为空！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            await base.TryCloseAsync(true);
        }
        #endregion

        #endregion
    }
}
