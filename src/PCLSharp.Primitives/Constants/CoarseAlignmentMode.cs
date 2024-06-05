using System;
using System.ComponentModel;
using System.Runtime.Serialization;

namespace PCLSharp.Primitives.Constants
{
    /// <summary>
    /// 粗配准模式
    /// </summary>
    [Serializable]
    [DataContract]
    public enum CoarseAlignmentMode
    {
        /// <summary>
        /// K-FPCS
        /// </summary>
        [EnumMember]
        [Description("K-FPCS")]
        KFPCS = 0,

        /// <summary>
        /// SAC-IA
        /// </summary>
        [EnumMember]
        [Description("SAC-IA")]
        SACIA = 1
    }
}
