using Autofac;
using Caliburn.Micro;
using PCLSharp.Client.ViewModels.HomeContext;
using SD.Infrastructure.WPF.Caliburn.Extensions;
using SD.IOC.Core.Extensions;
using SD.IOC.Core.Mediators;
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Threading;

namespace PCLSharp.Client
{
    /// <summary>
    /// Caliburn启动器
    /// </summary>
    public class Startup : BootstrapperBase
    {
        #region # 构造器

        #region 00.无参构造器
        /// <summary>
        /// 无参构造器
        /// </summary>
        public Startup()
        {
            this.Initialize();
        }
        #endregion

        #endregion

        #region # 事件

        #region 应用程序启动事件 —— override void OnStartup(object sender...
        /// <summary>
        /// 应用程序启动事件
        /// </summary>
        protected override async void OnStartup(object sender, StartupEventArgs e)
        {
            //启动屏幕
            SplashScreen splashScreen = new SplashScreen("Content/Images/PCL.png");
            splashScreen.Show(true);

            await base.DisplayRootViewForAsync<IndexViewModel>();
        }
        #endregion

        #region 应用程序异常事件 —— override void OnUnhandledException(object sender...
        /// <summary>
        /// 应用程序异常事件
        /// </summary>
        protected override void OnUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs eventArgs)
        {
            Exception exception = GetInnerException(eventArgs.Exception);
            eventArgs.Handled = true;

            //释放遮罩
            BusyExtension.GlobalIdle();

            //提示消息
            MessageBox.Show(exception.Message, "错误", MessageBoxButton.OK, MessageBoxImage.Error);
        }
        #endregion

        #region 应用程序退出事件 —— override void OnExit(object sender...
        /// <summary>
        /// 应用程序退出事件
        /// </summary>
        protected override void OnExit(object sender, EventArgs e)
        {
            ResolveMediator.Dispose();
        }
        #endregion

        #endregion

        #region # 方法

        #region 配置应用程序 —— override void Configure()
        /// <summary>
        /// 配置应用程序
        /// </summary>
        protected override void Configure()
        {
            //初始化依赖注入容器
            if (!ResolveMediator.ContainerBuilt)
            {
                ContainerBuilder containerBuilder = ResolveMediator.GetContainerBuilder();
                containerBuilder.RegisterConfigs();
                ResolveMediator.Build();
            }
        }
        #endregion

        #region 解析服务实例 —— override object GetInstance(Type service...
        /// <summary>
        /// 解析服务实例
        /// </summary>
        /// <param name="service">服务类型</param>
        /// <param name="key">键</param>
        /// <returns>服务实例</returns>
        protected override object GetInstance(Type service, string key)
        {
            object instance = ResolveMediator.Resolve(service);
            return instance;
        }
        #endregion

        #region 解析服务实例列表 —— override IEnumerable<object> GetAllInstances(Type service)
        /// <summary>
        /// 解析服务实例列表
        /// </summary>
        /// <param name="service">服务类型</param>
        /// <returns>服务实例列表</returns>
        protected override IEnumerable<object> GetAllInstances(Type service)
        {
            IEnumerable<object> instances = ResolveMediator.ResolveAll(service);
            return instances;
        }
        #endregion

        #region 获取内部异常 —— static Exception GetInnerException(Exception exception)
        /// <summary>
        /// 获取内部异常
        /// </summary>
        /// <param name="exception">异常</param>
        /// <returns>内部异常</returns>
        private static Exception GetInnerException(Exception exception)
        {
            if (exception.InnerException != null)
            {
                return GetInnerException(exception.InnerException);
            }

            return exception;
        }
        #endregion

        #endregion
    }
}
