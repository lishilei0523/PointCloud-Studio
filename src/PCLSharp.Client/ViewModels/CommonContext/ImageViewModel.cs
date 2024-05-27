using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.Caliburn.Base;
using System.Windows.Media;

namespace PCLSharp.Client.ViewModels.CommonContext
{
    /// <summary>
    /// 图像查看视图模型
    /// </summary>
    public class ImageViewModel : ScreenBase
    {
        #region # 字段及构造器

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public ImageViewModel()
        {

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

        #region 图像源 —— ImageSource ImageSource
        /// <summary>
        /// 图像源
        /// </summary>
        [DependencyProperty]
        public ImageSource ImageSource { get; set; }
        #endregion

        #endregion

        #region # 方法

        //Initializations

        #region 加载 —— void Load(string title, ImageSource imageSource)
        /// <summary>
        /// 加载
        /// </summary>
        /// <param name="title">标题</param>
        /// <param name="imageSource">图像源</param>
        public void Load(string title, ImageSource imageSource)
        {
            this.Title = title;
            this.ImageSource = imageSource;
        }
        #endregion

        #endregion
    }
}
