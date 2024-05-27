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

        #region # 添加PFH描述子 —— static void AddPFH(this Plot plot, IEnumerable<PFHSignature125F> descriptors)
        /// <summary>
        /// 添加PFH描述子
        /// </summary>
        /// <param name="plot">绘图器</param>
        /// <param name="descriptors">PFH描述子集</param>
        public static void AddPFH(this Plot plot, IEnumerable<PFHSignature125F> descriptors)
        {
            #region # 验证

            if (descriptors == null)
            {
                throw new ArgumentNullException(nameof(descriptors), "PFH描述子不可为空！");
            }

            #endregion

            double[] xs = Enumerable.Range(1, Constants.LengthOfPFH).Select(x => (double)x).ToArray();
            foreach (PFHSignature125F descriptor in descriptors)
            {
                double[] ys = descriptor.Features.Select(x => (double)x).ToArray();
                plot.Add.ScatterLine(xs, ys);
            }
        }
        #endregion

        #region # 添加FPFH描述子 —— static void AddFPFH(this Plot plot, IEnumerable<FPFHSignature33F> descriptors)
        /// <summary>
        /// 添加FPFH描述子
        /// </summary>
        /// <param name="plot">绘图器</param>
        /// <param name="descriptors">FPFH描述子集</param>
        public static void AddFPFH(this Plot plot, IEnumerable<FPFHSignature33F> descriptors)
        {
            #region # 验证

            if (descriptors == null)
            {
                throw new ArgumentNullException(nameof(descriptors), "FPFH描述子不可为空！");
            }

            #endregion

            double[] xs = Enumerable.Range(1, Constants.LengthOfFPFH).Select(x => (double)x).ToArray();
            foreach (FPFHSignature33F descriptor in descriptors)
            {
                double[] ys = descriptor.Features.Select(x => (double)x).ToArray();
                plot.Add.ScatterLine(xs, ys);
            }
        }
        #endregion

        #region # 添加3DSC描述子 —— static void Add3DSC(this Plot plot, IEnumerable<ShapeContext1980F> descriptors)
        /// <summary>
        /// 添加3DSC描述子
        /// </summary>
        /// <param name="plot">绘图器</param>
        /// <param name="descriptors">3DSC描述子集</param>
        public static void Add3DSC(this Plot plot, IEnumerable<ShapeContext1980F> descriptors)
        {
            #region # 验证

            if (descriptors == null)
            {
                throw new ArgumentNullException(nameof(descriptors), "3DSC描述子不可为空！");
            }

            #endregion

            double[] xs = Enumerable.Range(1, Constants.LengthOf3DSC).Select(x => (double)x).ToArray();
            foreach (ShapeContext1980F descriptor in descriptors)
            {
                double[] ys = descriptor.Features.Select(x => (double)x).ToArray();
                plot.Add.ScatterLine(xs, ys);
            }
        }
        #endregion

        #region # 添加SHOT描述子 —— static void AddSHOT(this Plot plot, IEnumerable<Shot352F> descriptors)
        /// <summary>
        /// 添加SHOT描述子
        /// </summary>
        /// <param name="plot">绘图器</param>
        /// <param name="descriptors">SHOT描述子集</param>
        public static void AddSHOT(this Plot plot, IEnumerable<Shot352F> descriptors)
        {
            #region # 验证

            if (descriptors == null)
            {
                throw new ArgumentNullException(nameof(descriptors), "SHOT描述子不可为空！");
            }

            #endregion

            double[] xs = Enumerable.Range(1, Constants.LengthOfSHOT).Select(x => (double)x).ToArray();
            foreach (Shot352F descriptor in descriptors)
            {
                double[] ys = descriptor.Features.Select(x => (double)x).ToArray();
                plot.Add.ScatterLine(xs, ys);
            }
        }
        #endregion
    }
}
