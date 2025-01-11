using Microsoft.VisualStudio.TestTools.UnitTesting;
using PCLSharp.Modules.Implements;
using PCLSharp.Modules.Interfaces;
using PCLSharp.Primitives.Models;

namespace PCLSharp.Tests.TestCases
{
    /// <summary>
    /// 点云通用操作测试
    /// </summary>
    [TestClass]
    public class CommonTests
    {
        #region # 测试估算质心 —— void TestEstimateCentroid()
        /// <summary>
        /// 测试估算质心
        /// </summary>
        [TestMethod]
        public void TestEstimateCentroid()
        {
            string filePath = "../../../../../assets/table_scene_lms400.pcd";

            ICloudFiles cloudFiles = new CloudFiles();
            Point3F[] cloud = cloudFiles.LoadPCD(filePath);

            ICloudCommon cloudCommon = new CloudCommon();
            Point3F centroid = cloudCommon.EstimateCentroid(cloud);

            Assert.AreEqual(centroid, new Point3F(-0.0994279459f, -0.307722867f, -1.3541435f));
        }
        #endregion
    }
}
