namespace PCLSharp.Primitives.Constants
{
    /// <summary>
    /// 程序集名称
    /// </summary>
    public static class AssemblyNames
    {
        /// <summary>
        /// 基元程序集
        /// </summary>
#if NET462_OR_GREATER
        public const string Primitives = "PCLSharp.Primitives.Native.dll";
#endif
#if NETSTANDARD2_0_OR_GREATER
        public const string Primitives = "PCLSharp.Primitives.Native";
#endif

        /// <summary>
        /// 模块程序集
        /// </summary>
#if NET462_OR_GREATER
        public const string Modules = "PCLSharp.Modules.Native.dll";
#endif
#if NETSTANDARD2_0_OR_GREATER
        public const string Modules = "PCLSharp.Modules.Native";
#endif
    }
}
