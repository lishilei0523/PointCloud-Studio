﻿using System;
using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Features
{
    /// <summary>
    /// SHOT描述子集
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Shot352Fs
    {
        /// <summary>
        /// 创建SHOT描述子集构造器
        /// </summary>
        /// <param name="descriptors">SHOT描述子集指针</param>
        /// <param name="length">长度</param>
        public Shot352Fs(IntPtr descriptors, int length)
            : this()
        {
            this.Descriptors = descriptors;
            this.Length = length;
        }

        /// <summary>
        /// SHOT描述子集指针
        /// </summary>
        public readonly IntPtr Descriptors;

        /// <summary>
        /// 长度
        /// </summary>
        public readonly int Length;
    }
}
