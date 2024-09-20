using HelixToolkit.Wpf.SharpDX;
using HelixToolkit.Wpf.SharpDX.Model.Scene;
using MathNet.Numerics.LinearAlgebra;
using PCLSharp.Modules.Interfaces;
using SD.Infrastructure.WPF.Caliburn.Aspects;
using SD.Infrastructure.WPF.ThreeDims.Extensions;
using SD.Infrastructure.WPF.ThreeDims.Visual3Ds;
using SD.Toolkits.Mathematics.Extensions;
using SharpDX;
using System;
using System.Diagnostics;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using DiffuseMaterial = HelixToolkit.Wpf.SharpDX.DiffuseMaterial;
using Point = System.Windows.Point;
using Pose = SD.Toolkits.Mathematics.Models.Pose;

namespace PCLSharp.Client.ViewModels.CommonContext
{
    /// <summary>
    /// 盒子裁剪视图模型
    /// </summary>
    public class CropBoxViewModel : PreviewViewModel
    {
        #region # 字段及构造器

        /// <summary>
        /// 是否选中长方体
        /// </summary>
        private bool _isSelectedBox;

        /// <summary>
        /// 选中2D点
        /// </summary>
        private Point? _selectedPoint2D;

        /// <summary>
        /// 依赖注入构造器
        /// </summary>
        public CropBoxViewModel(ICloudCommon cloudCommon)
            : base(cloudCommon)
        {
        }

        #endregion

        #region # 属性

        #region 长方体 —— BoxVisual3D BoundingBox
        /// <summary>
        /// 长方体
        /// </summary>
        [DependencyProperty]
        public BoxVisual3D BoundingBox { get; set; }
        #endregion

        #endregion

        #region # 方法

        //Initializations

        #region 加载 —— override void Load(PointGeometry3D pointCloud)
        /// <summary>
        /// 加载
        /// </summary>
        public override void Load(PointGeometry3D pointCloud)
        {
            base.Load(pointCloud);

            this.BoundingBox = new BoxVisual3D();
            this.BoundingBox.Length = this.PointCloud.Bound.Width;
            this.BoundingBox.Width = this.PointCloud.Bound.Height;
            this.BoundingBox.Height = this.PointCloud.Bound.Depth;
            Matrix3D boxMatrix = new Matrix3D
            {
                OffsetX = this.PointCloud.Bound.Center.X,
                OffsetY = this.PointCloud.Bound.Center.Y,
                OffsetZ = this.PointCloud.Bound.Center.Z
            };
            this.BoundingBox.Transform = new MatrixTransform3D(boxMatrix);
            this.BoundingBox.Material = new DiffuseMaterial
            {
                DiffuseColor = new Color4(1, 0, 0, 0.25f)
            };
        }
        #endregion


        //Actions

        #region 剪裁 —— void Crop()
        /// <summary>
        /// 剪裁
        /// </summary>
        public void Crop()
        {
            Vector3Collection positions = new Vector3Collection();
            foreach (Vector3 position in this.PointCloud.Positions)
            {
                if (!this.BoundingBox.ContainsPoint(position))
                {
                    positions.Add(position);
                }
            }
            this.PointCloud = new PointGeometry3D
            {
                Positions = positions
            };
        }
        #endregion

        #region 保留 —— void Reserve()
        /// <summary>
        /// 保留
        /// </summary>
        public void Reserve()
        {
            Vector3Collection positions = new Vector3Collection();
            foreach (Vector3 position in this.PointCloud.Positions)
            {
                if (this.BoundingBox.ContainsPoint(position))
                {
                    positions.Add(position);
                }
            }
            this.PointCloud = new PointGeometry3D
            {
                Positions = positions
            };
        }
        #endregion


        //Events

        #region 鼠标左键按下事件 —— void ViewportOnMouseLeftDown(Viewport3DX viewport3D, MouseButtonEventArgs eventArgs)
        /// <summary>
        /// 鼠标左键按下事件
        /// </summary>
        public void ViewportOnMouseLeftDown(Viewport3DX viewport3D, MouseButtonEventArgs eventArgs)
        {
            Point mousePos2D = eventArgs.GetPosition(viewport3D);
            Point3D mousePos3D;
            Element3D visual3D;
            bool success = viewport3D.FindNearest(mousePos2D, out mousePos3D, out Vector3D _, out visual3D, out SceneNode _);

            //获得焦点
            if (success && Keyboard.IsKeyDown(Key.LeftShift))
            {
                viewport3D.LookAt(mousePos3D, 200);

                eventArgs.Handled = true;
                return;
            }
            //选中元素
            if (success && visual3D is BoxVisual3D)
            {
                this._isSelectedBox = true;
                this._selectedPoint2D = mousePos2D;

                eventArgs.Handled = true;
                return;
            }
        }
        #endregion

        #region 鼠标移动事件 —— void ViewportOnMouseMove(Viewport3DX viewport3D, MouseEventArgs eventArgs)
        /// <summary>
        /// 鼠标移动事件
        /// </summary>
        public void ViewportOnMouseMove(Viewport3DX viewport3D, MouseEventArgs eventArgs)
        {
            if (this._isSelectedBox)
            {
                //计算模型位置
                Matrix3D rtMatrix3D = this.BoundingBox.Transform.Value;         //模型的变换矩阵
                Point3D oldVisualPos3D = rtMatrix3D.GetLocation();              //模型位置

                //计算模型新位置
                Point mousePos2D = eventArgs.GetPosition(viewport3D);           //鼠标位置
                Ray ray3D = viewport3D.UnProject(mousePos2D);                   //反透射
                Vector3D lookDirction = viewport3D.Camera.LookDirection;        //相机方向
                Vector3 position = oldVisualPos3D.ToVector3();
                Vector3 normal = lookDirction.ToVector3();
                bool success = ray3D.PlaneIntersection(position, normal, out Vector3 newVisualPos3D);//移动平面上的交点

                //缩放
                if (success && eventArgs.LeftButton == MouseButtonState.Pressed && Keyboard.IsKeyDown(Key.LeftCtrl))
                {
                    //设置光标
                    Mouse.OverrideCursor = Cursors.SizeNWSE;

                    //计算新尺寸
                    if (viewport3D.Camera.LookDirection.X.Equals(0) && viewport3D.Camera.LookDirection.Y.Equals(0))
                    {
                        double diffX = Math.Abs(newVisualPos3D.X - oldVisualPos3D.X);
                        double diffY = Math.Abs(newVisualPos3D.Y - oldVisualPos3D.Y);
                        this.BoundingBox.Length = diffX * 2;
                        this.BoundingBox.Width = diffY * 2;
                    }
                    else if (viewport3D.Camera.LookDirection.X.Equals(0) && viewport3D.Camera.LookDirection.Z.Equals(0))
                    {
                        double diffX = Math.Abs(newVisualPos3D.X - oldVisualPos3D.X);
                        double diffZ = Math.Abs(newVisualPos3D.Z - oldVisualPos3D.Z);
                        this.BoundingBox.Length = diffX * 2;
                        this.BoundingBox.Height = diffZ * 2;
                    }
                    else if (viewport3D.Camera.LookDirection.Y.Equals(0) && viewport3D.Camera.LookDirection.Z.Equals(0))
                    {
                        double diffY = Math.Abs(newVisualPos3D.Y - oldVisualPos3D.Y);
                        double diffZ = Math.Abs(newVisualPos3D.Z - oldVisualPos3D.Z);
                        this.BoundingBox.Width = diffY * 2;
                        this.BoundingBox.Height = diffZ * 2;
                    }
                    else
                    {
                        Trace.WriteLine("不允许！");
                    }

                    eventArgs.Handled = true;
                    return;
                }
                //旋转
                if (success && eventArgs.LeftButton == MouseButtonState.Pressed && Keyboard.IsKeyDown(Key.LeftAlt))
                {
                    //设置光标
                    Mouse.OverrideCursor = Cursors.ScrollWE;

                    double angle = mousePos2D.X - this._selectedPoint2D!.Value.X + mousePos2D.Y - this._selectedPoint2D!.Value.Y;
                    Matrix<double> rtMatrix = TransformExtension.ToMatrix(rtMatrix3D);
                    Pose oldPose = rtMatrix.ToPose();
                    Pose newPose;
                    if (viewport3D.Camera.LookDirection.X.Equals(0) && viewport3D.Camera.LookDirection.Y.Equals(0))
                    {
                        //绕Z轴转
                        newPose = new Pose(oldPose.X, oldPose.Y, oldPose.Z, oldPose.RX, oldPose.RY, angle);
                    }
                    else if (viewport3D.Camera.LookDirection.X.Equals(0) && viewport3D.Camera.LookDirection.Z.Equals(0))
                    {
                        //绕Y轴转
                        newPose = new Pose(oldPose.X, oldPose.Y, oldPose.Z, oldPose.RX, angle, oldPose.RZ);
                    }
                    else if (viewport3D.Camera.LookDirection.Y.Equals(0) && viewport3D.Camera.LookDirection.Z.Equals(0))
                    {
                        //绕X轴转
                        newPose = new Pose(oldPose.X, oldPose.Y, oldPose.Z, angle, oldPose.RY, oldPose.RZ);
                    }
                    else
                    {
                        Trace.WriteLine("不允许！");
                        eventArgs.Handled = true;
                        return;
                    }

                    Matrix<double> newMatrix = newPose.ToRotationTranslationMatrix();
                    Matrix3D newMatrix3D = newMatrix.ToMatrix3D();
                    this.BoundingBox.Transform = new MatrixTransform3D(newMatrix3D);

                    eventArgs.Handled = true;
                    return;
                }
                //平移
                if (success && eventArgs.LeftButton == MouseButtonState.Pressed)
                {
                    //设置光标
                    Mouse.OverrideCursor = Cursors.Hand;

                    //移动新位置
                    rtMatrix3D.OffsetX = newVisualPos3D.X;
                    rtMatrix3D.OffsetY = newVisualPos3D.Y;
                    rtMatrix3D.OffsetZ = newVisualPos3D.Z;
                    this.BoundingBox.Transform = new MatrixTransform3D(rtMatrix3D);

                    eventArgs.Handled = true;
                    return;
                }
            }
        }
        #endregion

        #region 鼠标松开事件 —— void ViewportOnMouseUp()
        /// <summary>
        /// 鼠标松开事件
        /// </summary>
        public void ViewportOnMouseUp()
        {
            //设置光标
            Mouse.OverrideCursor = Cursors.Arrow;

            //清空选中
            this._isSelectedBox = false;
            this._selectedPoint2D = null;
        }
        #endregion

        #endregion
    }
}
