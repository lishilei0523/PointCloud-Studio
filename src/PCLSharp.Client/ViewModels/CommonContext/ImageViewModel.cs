using Caliburn.Micro;
using Microsoft.Win32;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.IO;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media.Imaging;

namespace PCLSharp.Client.ViewModels.CommonContext
{
    /// <summary>
    /// 图像查看视图模型
    /// </summary>
    public class ImageViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 窗体管理器
        /// </summary>
        private readonly IWindowManager _windowManager;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public ImageViewModel(IWindowManager windowManager)
        {
            this._windowManager = windowManager;
        }

        #endregion

        #region # 属性

        #region 标题 —— string Title
        /// <summary>
        /// 标题
        /// </summary>
        [DependencyProperty]
        public string Title { get; set; }
        #endregion

        #region 图像源 —— BitmapSource Image
        /// <summary>
        /// 图像源
        /// </summary>
        [DependencyProperty]
        public BitmapSource Image { get; set; }
        #endregion

        #endregion

        #region # 方法

        #region 加载 —— void Load(BitmapSource image...
        /// <summary>
        /// 加载
        /// </summary>
        /// <param name="image">图像</param>
        /// <param name="title">标题</param>
        public void Load(BitmapSource image, string title = "查看图像")
        {
            this.Image = image;
            this.Title = title;
        }
        #endregion

        #region 另存为图像 —— async void SaveAsImage()
        /// <summary>
        /// 另存为图像
        /// </summary>
        public async void SaveAsImage()
        {
            #region # 验证

            if (this.Image == null)
            {
                MessageBox.Show("图像未加载！", "错误", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            #endregion

            SaveFileDialog saveFileDialog = new SaveFileDialog
            {
                FileName = this.Title,
                Filter = "(*.jpg)|*.jpg",
                AddExtension = true,
                RestoreDirectory = true
            };
            if (saveFileDialog.ShowDialog() == true)
            {
                this.Busy();

                BitmapEncoder bitmapEncoder = new JpegBitmapEncoder();
                BitmapFrame bitmapFrame = BitmapFrame.Create(this.Image);
                bitmapEncoder.Frames.Add(bitmapFrame);
                using FileStream outputStream = File.OpenWrite(saveFileDialog.FileName);
                await Task.Run(() => bitmapEncoder.Save(outputStream));

                this.Idle();
                this.ToastSuccess("保存成功！");
            }
        }
        #endregion

        #endregion
    }
}
