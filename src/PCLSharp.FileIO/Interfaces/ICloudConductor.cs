using PCLSharp.Primitives.Models;
using System.Collections.Generic;

namespace PCLSharp.FileIO.Interfaces
{
    /// <summary>
    /// 点云读写器接口
    /// </summary>
    public interface ICloudConductor
    {
        #region # 加载PCD文件 —— Point3F[] LoadPCD(string filePath)
        /// <summary>
        /// 加载PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3F[] LoadPCD(string filePath);
        #endregion

        #region # 加载PLY文件 —— Point3F[] LoadPLY(string filePath)
        /// <summary>
        /// 加载PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3F[] LoadPLY(string filePath);
        #endregion

        #region # 加载OBJ文件 —— Point3F[] LoadOBJ(string filePath)
        /// <summary>
        /// 加载OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3F[] LoadOBJ(string filePath);
        #endregion

        #region # 保存PCD文本文件 —— void SaveTextPCD(IEnumerable<Point3F> points, string filePath)
        /// <summary>
        /// 保存PCD文本文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveTextPCD(IEnumerable<Point3F> points, string filePath);
        #endregion

        #region # 保存PCD二进制文件 —— void SaveBinaryPCD(IEnumerable<Point3F> points, string filePath)
        /// <summary>
        /// 保存PCD二进制文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveBinaryPCD(IEnumerable<Point3F> points, string filePath);
        #endregion

        #region # 保存PLY文本文件 —— void SaveTextPLY(IEnumerable<Point3F> points, string filePath)
        /// <summary>
        /// 保存PLY文本文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveTextPLY(IEnumerable<Point3F> points, string filePath);
        #endregion

        #region # 保存PLY二进制文件 —— void SaveBinaryPLY(IEnumerable<Point3F> points, string filePath)
        /// <summary>
        /// 保存PLY二进制文件
        /// </summary>
        /// <param name="points">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveBinaryPLY(IEnumerable<Point3F> points, string filePath);
        #endregion
    }
}
