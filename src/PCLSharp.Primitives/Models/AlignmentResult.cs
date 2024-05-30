using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 配准结果
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct AlignmentResult
    {
        /// <summary>
        /// 创建配准结果构造器
        /// </summary>
        /// <param name="hasConverged">是否收敛</param>
        /// <param name="fitnessScore">拟合分数</param>
        /// <param name="matrix">RT矩阵</param>
        public AlignmentResult(bool hasConverged, float fitnessScore, float[] matrix)
        {
            this.HasConverged = hasConverged;
            this.FitnessScore = fitnessScore;
            this.Matrix = matrix;
        }

        /// <summary>
        /// 是否收敛
        /// </summary>
        public readonly bool HasConverged;

        /// <summary>
        /// 拟合分数
        /// </summary>
        public readonly float FitnessScore;

        /// <summary>
        /// RT矩阵
        /// </summary>
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public readonly float[] Matrix;
    }
}
