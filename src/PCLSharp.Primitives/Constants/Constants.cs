// ReSharper disable once CheckNamespace
namespace PCLSharp
{
    /// <summary>
    /// 常量字典
    /// </summary>
    public static class Constants
    {
        /// <summary>
        /// 耗时格式
        /// </summary>
        public const string DurationFormat = @"ss\.fff";

        /// <summary>
        /// 耗时格式
        /// </summary>
        public const string MatrixFormat = "F3";

        /// <summary>
        /// X轴
        /// </summary>
        public const string AxisX = "x";

        /// <summary>
        /// Y轴
        /// </summary>
        public const string AxisY = "y";

        /// <summary>
        /// Z轴
        /// </summary>
        public const string AxisZ = "z";

        /// <summary>
        /// PCD格式点云
        /// </summary>
        public const string PCD = ".pcd";

        /// <summary>
        /// PLY格式点云
        /// </summary>
        public const string PLY = ".ply";

        /// <summary>
        /// OBJ格式点云
        /// </summary>
        public const string OBJ = ".obj";

        /// <summary>
        /// 打开点云文件扩展名过滤器
        /// </summary>
        public const string OpenCloudExtFilter = "(*.pcd)|*.pcd|(*.ply)|*.ply|(*.obj)|*.obj";

        /// <summary>
        /// 保存点云文件扩展名过滤器
        /// </summary>
        public const string SaveCloudExtFilter = "(*.pcd)|*.pcd|(*.ply)|*.ply";

        /// <summary>
        /// NARF描述子长度
        /// </summary>
        public const int LengthOfNARF = 36;

        /// <summary>
        /// PFH描述子长度
        /// </summary>
        public const int LengthOfPFH = 125;

        /// <summary>
        /// FPFH描述子长度
        /// </summary>
        public const int LengthOfFPFH = 33;

        /// <summary>
        /// 3DSC描述子长度
        /// </summary>
        public const int LengthOf3DSC = 1980;

        /// <summary>
        /// SHOT描述子长度
        /// </summary>
        public const int LengthOfSHOT = 352;
    }
}
