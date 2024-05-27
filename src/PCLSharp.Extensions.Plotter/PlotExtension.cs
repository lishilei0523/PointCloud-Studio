using PCLSharp.Primitives.Features;
using ScottPlot;
using System;
using System.Collections.Generic;
using System.Linq;

namespace PCLSharp.Extensions.Plotter
{
    /// <summary>
    /// 绘图扩展
    /// </summary>
    public static class PlotExtension
    {
        #region # 添加NARF描述子 —— static void AddNARF(this Plot plot, IEnumerable<Narf36F> descriptors)
        /// <summary>
        /// 添加NARF描述子
        /// </summary>
        /// <param name="plot">绘图器</param>
        /// <param name="descriptors">NARF描述子集</param>
        public static void AddNARF(this Plot plot, IEnumerable<Narf36F> descriptors)
        {
            #region # 验证

            if (descriptors == null)
            {
                throw new ArgumentNullException(nameof(descriptors), "NARF描述子不可为空！");
            }

            #endregion

            double[] xs = Enumerable.Range(1, Constants.LengthOfNARF).Select(x => (double)x).ToArray();
            foreach (Narf36F descriptor in descriptors)
            {
                double[] ys = descriptor.Features.Select(x => (double)x).ToArray();
                plot.Add.ScatterLine(xs, ys);
            }
        }
        #endregion
    }
}
