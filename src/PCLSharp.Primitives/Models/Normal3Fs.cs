using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 法向量集结构体
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Normal3Fs
    {
        /// <summary>
        /// 创建法向量集结构体构造器
        /// </summary>
        /// <param name="normals">法向量集指针</param>
        /// <param name="length">长度</param>
        public Normal3Fs(IntPtr normals, int length)
            : this()
        {
            this.Normals = normals;
            this.Length = length;
        }

        /// <summary>
        /// 法向量集指针
        /// </summary>
        public readonly IntPtr Normals;

        /// <summary>
        /// 长度
        /// </summary>
        public readonly int Length;
    }
}
