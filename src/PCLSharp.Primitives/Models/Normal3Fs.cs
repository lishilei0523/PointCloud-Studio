using System;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 法向量集结构体
    /// </summary>
    public struct Normal3Fs
    {
        /// <summary>
        /// 法向量集指针
        /// </summary>
        public IntPtr Normals;

        /// <summary>
        /// 长度
        /// </summary>
        public int Length;
    }
}
