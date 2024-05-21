namespace PCLSharp.Primitives.Constants
{
    /// <summary>
    /// 程序集名称
    /// </summary>
    public static class AssemblyNames
    {
        /// <summary>
        /// 滤波程序集
        /// </summary>
        public const string Filters =
#if NET462_OR_GREATER
            "PCLSharp.Filters.Native.dll";
#endif
#if NETSTANDARD2_0_OR_GREATER
            "PCLSharp.Filters.Native";
#endif
    }
}
