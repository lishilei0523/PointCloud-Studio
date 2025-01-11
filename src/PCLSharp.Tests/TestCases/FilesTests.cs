using Microsoft.VisualStudio.TestTools.UnitTesting;
using PCLSharp.Modules.Implements;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;

namespace PCLSharp.Tests.TestCases
{
    /// <summary>
    /// 点云读写测试
    /// </summary>
    [TestClass]
    public class FilesTests
    {
        #region # 测试加载PCD文件 —— void TestLoadPCD()
        /// <summary>
        /// 测试加载PCD文件
        /// </summary>
        [TestMethod]
        public void TestLoadPCD()
        {
            string filePath = "../../../../../assets/table_scene_lms400.pcd";
            ICloudFiles cloudFiles = new CloudFiles();
            Point3F[] cloud = cloudFiles.LoadPCD(filePath);

            Assert.AreEqual(cloud.Length, 460400);
        }
        #endregion
    }
}
