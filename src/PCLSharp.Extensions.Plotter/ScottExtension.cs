using ScottPlot;
using SkiaSharp;
using System;
using System.Reflection;

namespace PCLSharp.Extensions.Plotter
{
    /// <summary>
    /// ScottPlot扩展
    /// </summary>
    public static class ScottExtension
    {
        #region # 获取SKImage —— static SKImage GetSKImage(this Plot plot, int width, int height)
        /// <summary>
        /// 获取SKImage
        /// </summary>
        /// <param name="plot">绘图器</param>
        /// <param name="width">图像宽度</param>
        /// <param name="height">图像高度</param>
        /// <returns>SKImage</returns>
        public static SKImage GetSKImage(this Plot plot, int width = 1024, int height = 768)
        {
            Image image = plot.GetImage(width, height);
            Type imageType = image.GetType();
            PropertyInfo propertyInfo = imageType.GetProperty(nameof(SKImage), BindingFlags.Instance | BindingFlags.NonPublic);
            SKImage skImage = (SKImage)propertyInfo!.GetValue(image);

            return skImage;
        }
        #endregion
    }
}
