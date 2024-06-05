using System;
using System.ComponentModel;
using System.Runtime.Serialization;

namespace PCLSharp.Primitives.Constants
{
    /// <summary>
    /// 精配准模式
    /// </summary>
    [Serializable]
    [DataContract]
    public enum FineAlignmentMode
    {
        /// <summary>
        /// 点到点
        /// </summary>
        [EnumMember]
        [Description("点到点")]
        PointToPoint = 0,

        /// <summary>
        /// 点到面
        /// </summary>
        [EnumMember]
        [Description("点到面")]
        PointToPlane = 1,

        /// <summary>
        /// Generalized-ICP
        /// </summary>
        [EnumMember]
        [Description("GICP")]
        GICP = 2
    }
}
