using System.Runtime.InteropServices;

namespace PCLSharp.Primitives.Models
{
    /// <summary>
    /// 法向量
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct Normal3F
    {
        /// <summary>
        /// 创建法向量构造器
        /// </summary>
        /// <param name="nx">法向量X坐标</param>
        /// <param name="ny">法向量Y坐标</param>
        /// <param name="nz">法向量Z坐标</param>
        public Normal3F(float nx, float ny, float nz)
            : this()
        {
            this.NX = nx;
            this.NY = ny;
            this.NZ = nz;
        }

        /// <summary>
        /// 法向量X坐标
        /// </summary>
        public readonly float NX;

        /// <summary>
        /// 法向量Y坐标
        /// </summary>
        public readonly float NY;

        /// <summary>
        /// 法向量Z坐标
        /// </summary>
        public readonly float NZ;

        /// <summary>
        /// 是否是非数值
        /// </summary>
        public bool IsNaN()
        {
            if (this.NX.Equals(float.NaN) || this.NY.Equals(float.NaN) || this.NZ.Equals(float.NaN))
            {
                return true;
            }

            return false;
        }
    }
}
