using PCLSharp.Primitives.Models;
using System.Collections.Generic;

namespace PCLSharp.Modules.Interfaces
{
    /// <summary>
    /// 点云读写接口
    /// </summary>
    public interface ICloudFiles
    {
        #region # 加载PCD文件 —— Point3F[] LoadPCD(string filePath)
        /// <summary>
        /// 加载PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3F[] LoadPCD(string filePath);
        #endregion

        #region # 加载法向量PCD文件 —— Point3Normal3[] LoadNormalPCD(string filePath)
        /// <summary>
        /// 加载法向量PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3Normal3[] LoadNormalPCD(string filePath);
        #endregion

        #region # 加载颜色PCD文件 —— Point3Color4[] LoadColorPCD(string filePath)
        /// <summary>
        /// 加载颜色PCD文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3Color4[] LoadColorPCD(string filePath);
        #endregion

        #region # 加载PLY文件 —— Point3F[] LoadPLY(string filePath)
        /// <summary>
        /// 加载PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3F[] LoadPLY(string filePath);
        #endregion

        #region # 加载法向量PLY文件 —— Point3Normal3[] LoadNormalPLY(string filePath)
        /// <summary>
        /// 加载法向量PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3Normal3[] LoadNormalPLY(string filePath);
        #endregion

        #region # 加载颜色PLY文件 —— Point3Color4[] LoadColorPLY(string filePath)
        /// <summary>
        /// 加载颜色PLY文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3Color4[] LoadColorPLY(string filePath);
        #endregion

        #region # 加载OBJ文件 —— Point3F[] LoadOBJ(string filePath)
        /// <summary>
        /// 加载OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3F[] LoadOBJ(string filePath);
        #endregion

        #region # 加载法向量OBJ文件 —— Point3Normal3[] LoadNormalOBJ(string filePath)
        /// <summary>
        /// 加载法向量OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3Normal3[] LoadNormalOBJ(string filePath);
        #endregion

        #region # 加载颜色OBJ文件 —— Point3Color4[] LoadColorOBJ(string filePath)
        /// <summary>
        /// 加载颜色OBJ文件
        /// </summary>
        /// <param name="filePath">文件路径</param>
        /// <returns>点集</returns>
        Point3Color4[] LoadColorOBJ(string filePath);
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

        #region # 保存法向量PCD文本文件 —— void SaveNormalTextPCD(IEnumerable<Point3Normal3> pointNormals...
        /// <summary>
        /// 保存法向量PCD文本文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveNormalTextPCD(IEnumerable<Point3Normal3> pointNormals, string filePath);
        #endregion

        #region # 保存法向量PCD二进制文件 —— void SaveNormalBinaryPCD(IEnumerable<Point3Normal3> pointNormals...
        /// <summary>
        /// 保存法向量PCD二进制文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveNormalBinaryPCD(IEnumerable<Point3Normal3> pointNormals, string filePath);
        #endregion

        #region # 保存颜色PCD文本文件 —— void SaveColorTextPCD(IEnumerable<Point3Color4> pointColors...
        /// <summary>
        /// 保存颜色PCD文本文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveColorTextPCD(IEnumerable<Point3Color4> pointColors, string filePath);
        #endregion

        #region # 保存颜色PCD二进制文件 —— void SaveColorBinaryPCD(IEnumerable<Point3Color4> pointColors...
        /// <summary>
        /// 保存颜色PCD二进制文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveColorBinaryPCD(IEnumerable<Point3Color4> pointColors, string filePath);
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

        #region # 保存法向量PLY文本文件 —— void SaveNormalTextPLY(IEnumerable<Point3Normal3> pointNormals...
        /// <summary>
        /// 保存法向量PLY文本文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveNormalTextPLY(IEnumerable<Point3Normal3> pointNormals, string filePath);
        #endregion

        #region # 保存法向量PLY二进制文件 —— void SaveNormalBinaryPLY(IEnumerable<Point3Normal3> pointNormals...
        /// <summary>
        /// 保存法向量PLY二进制文件
        /// </summary>
        /// <param name="pointNormals">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveNormalBinaryPLY(IEnumerable<Point3Normal3> pointNormals, string filePath);
        #endregion

        #region # 保存颜色PLY文本文件 —— void SaveColorTextPLY(IEnumerable<Point3Color4> pointColors...
        /// <summary>
        /// 保存颜色PLY文本文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveColorTextPLY(IEnumerable<Point3Color4> pointColors, string filePath);
        #endregion

        #region # 保存颜色PLY二进制文件 —— void SaveColorBinaryPLY(IEnumerable<Point3Color4> pointColors...
        /// <summary>
        /// 保存颜色PLY二进制文件
        /// </summary>
        /// <param name="pointColors">点集</param>
        /// <param name="filePath">文件路径</param>
        void SaveColorBinaryPLY(IEnumerable<Point3Color4> pointColors, string filePath);
        #endregion
    }
}
