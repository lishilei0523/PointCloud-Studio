namespace PCLSharp.Primitives.Constants
{
    /// <summary>
    /// 程序集名称
    /// </summary>
    public static class AssemblyNames
    {
        /// <summary>
        /// 文件读写程序集
        /// </summary>
#if NET462_OR_GREATER
        public const string FileIO = "PCLSharp.FileIO.Native.dll";
#endif
#if NETSTANDARD2_0_OR_GREATER
        public const string FileIO = "PCLSharp.FileIO.Native";
#endif

        /// <summary>
        /// 滤波程序集
        /// </summary>
#if NET462_OR_GREATER
        public const string Filters = "PCLSharp.Filters.Native.dll";
#endif
#if NETSTANDARD2_0_OR_GREATER
        public const string Filters = "PCLSharp.Filters.Native";
#endif
    }
}
