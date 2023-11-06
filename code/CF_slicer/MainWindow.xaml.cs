using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Shapes;
using System.Windows.Media;

// File Dialog
using System.IO;
using Microsoft.Win32;

// Helix
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;

// Clipper
using Clipper2Lib;

// Classes
using static CF_slicer.Surface;
using static CF_slicer.SliceGenerator;
using static CF_slicer.Model;
using static CF_slicer.GCodeGenerator;

namespace CF_slicer
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    { 
        private int sliceIndex;
        private int sliceCount;
        
        private double sliceHeight;
        private double nozzleTickness;
        private int totalShells;
        private bool support;

        private int filamentTemp;
        private int bedTemp;
        private int printSpeed;
        private int moveSpeed;

        private bool IsMouseDown;
        private Vector offsetVector;
        private Point initialPos;

        private double canvasScale = 1;

        private List<Color> colors = new List<Color>(){
            Colors.Red,
            Colors.Blue,
            Colors.Green,
            Colors.Yellow,
            Colors.Orange,
            Colors.Blue,
            Colors.Purple
        };

        private Surface m_surface;
        private SliceGenerator m_sliceGenerator;
        private Model m_model;
        private GCodeGenerator m_gCodeGenerator;

        public MainWindow()
        {
            // Need to be initialized before components
            m_surface = new Surface();
            m_sliceGenerator = new SliceGenerator();
            m_model = new Model();
            m_gCodeGenerator = new GCodeGenerator();

            InitializeComponent();

            LoadModel("../../models/test_models/holes(easy).stl");
 
            //Adding a gesture here
            viewport3D.RotateGesture = new MouseGesture(MouseAction.LeftClick);
        }

      

        /// <summary>
        /// Loads given model and resets slices
        /// </summary>
        /// <param name="filePath"></param>
        private void LoadModel(string filePath)
        {
            const double sOffset = 10;

            m_model.LoadModel(filePath);

            // Put mesh in Helix 3D visualizer
            visualizedMesh.TriangleIndices = m_model.GetIndices();
            visualizedMesh.Positions = m_model.GetVertices();
            visualizedMesh.Normals = m_model.GetNormals();

            Rect3D bounds = m_model.GetBounds();

            Console.WriteLine("Bounds: {0} {1} {2} {3} {4} {5}", bounds.X, bounds.Y, bounds.Z, bounds.SizeX, bounds.SizeY, bounds.SizeX);
            
            m_surface.Initialize(surface, bounds.Z, 
                                bounds.X - (sOffset / 2), bounds.Y - (sOffset  / 2),
                                bounds.SizeX + sOffset, bounds.SizeY + sOffset);
            
            m_sliceGenerator.Initialize(bounds.Z, bounds.X, bounds.Y, 
                                        bounds.SizeX, bounds.SizeY, 
                                        m_model.GetVertices(), m_model.GetIndices());

            // Zoom to right distance
            viewport3D.ZoomExtents();

            // Starting properties
            UpdateVars();
            UpdateSliceCount();
            UpdateSliderProperties();
            ResetCheckboxSupport();

            // Zoom / Translate canvas to right spot
            InitialCanvasProperties();

            // Reset slices
            ResetSlices();
        }

        private void ResetSlices()
        {
            // Reset generate slices button
            Btn_GenerateSlices.ClearValue(Button.BackgroundProperty);

            // Clear canvas
            Canvas.Children.Clear();
        }

        private void Btn_OpenFile_Click(object sender, RoutedEventArgs e)
        {
            OpenFileDialog openFileDialog = new OpenFileDialog();
            openFileDialog.Filter = "STL files (*.stl)|*.stl";
            if (openFileDialog.ShowDialog() == true)
                LoadModel(openFileDialog.FileName);
        }

        private void Btn_GenerateSlices_Click(object sender, RoutedEventArgs e)
        {
            UpdateVars();
            UpdateSliceCount();
            UpdateSliderProperties();

            m_sliceGenerator.GenerateSlices();
            Btn_SlicePostProcess_Click(sender, e);
        }

        private void Btn_SlicePostProcess_Click(object sender, RoutedEventArgs e)
        {
            m_sliceGenerator.ReprocessSlices(support);

            // Redraw contour
            DrawSlices();
        }

        private void Btn_GenerateGCode_Click(object sender, RoutedEventArgs e)
        {
            UpdateVars();
            m_gCodeGenerator.GenerateGCode(m_sliceGenerator.GetPaths(), sliceHeight, 1.0, nozzleTickness);
            m_gCodeGenerator.WriteToFile("../../gcode/gtest.gcode");
        }

        private void Canvas_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            MatrixTransform matTrans = Canvas.RenderTransform as MatrixTransform;
            Point pos1 = e.GetPosition(Canvas_Control);

            double scale = e.Delta > 0 ? 1.1 : 1 / 1.1;
            canvasScale *= scale;

            Label_Scale.Content = "Scale: " + Math.Round(canvasScale, 2);

            Matrix mat = matTrans.Matrix;
            mat.ScaleAt(scale, scale, pos1.X, pos1.Y);
            matTrans.Matrix = mat;
            Canvas.RenderTransform = matTrans;
            e.Handled = true;
        }

        private void Canvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            IsMouseDown = true;
            initialPos = Mouse.GetPosition(this);
        }

        private void Canvas_MouseMove(object sender, MouseEventArgs e)
        {
            if (!IsMouseDown) return;

            MatrixTransform matTrans = Canvas.RenderTransform as MatrixTransform;

            Point newPos = Mouse.GetPosition(this);
            offsetVector = newPos - initialPos;
            initialPos = newPos;

            Matrix mat = matTrans.Matrix;
            mat.Translate(offsetVector.X, offsetVector.Y);
            matTrans.Matrix = mat;
            Canvas.RenderTransform = matTrans;
        }

        private void Canvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            IsMouseDown = false;
        }

        private void Canvas_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            MatrixTransform matTrans = Canvas.RenderTransform as MatrixTransform;
            Matrix mat = matTrans.Matrix;
            Console.WriteLine(mat.ToString());
        }

        private void Index_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            sliceIndex = (int)e.NewValue;

            // Change slice index label
            Label_SliceIndex.Content = "Slice index: " + sliceIndex;

            double minZ = m_sliceGenerator.GetMinZ();
            double planeZ = m_sliceGenerator.CalcPlaneZ(sliceIndex);

            //Console.WriteLine("Calculated surface Z: {0} {1} {2}", minZ, planeZ, planeZ - minZ);
           
            // Move surface (mesh always starts at 0 in visualisation -> start surface from 0)
            m_surface.MoveSurface(planeZ - minZ);

            // Draw contour
            DrawSlices();
        }
        private void Nozzle_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            nozzleTickness = (double)e.NewValue;
            m_sliceGenerator.SetNozzleThickness(nozzleTickness);

            // Change label
            nozzle_thickness.Content = "Nozzle thickness: " + nozzleTickness;
        }
        private void Shell_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            totalShells = (int)e.NewValue;
            m_sliceGenerator.SetTotalShells(totalShells);

            // Change label
            total_shells.Content = "Total shells: " + totalShells;
        }
        private void CheckBox_Support_Changed(object sender, RoutedEventArgs e)
        {
            support = (bool)CheckBox_Support.IsChecked;
            Console.WriteLine("Support enabled: " + support);
        }

        private void DrawSlices()
        {
            Canvas.Children.Clear();
            if (m_sliceGenerator.TotalSlices() != 0)
            {
                SliceInfo slice = m_sliceGenerator.GetSliceInfo(sliceIndex);

                // draw slices
                AddPathsToCanvas(slice.shells, false, Colors.Blue);
                AddPathsToCanvas(slice.contour, false, Colors.Blue);
                AddPathsToCanvas(slice.infill, true, Colors.Orange);
                AddPathsToCanvas(slice.floorsAndRoofs, true, Colors.Green);
                AddPathsToCanvas(slice.supportOpen, true, Colors.Red);
                AddPathsToCanvas(slice.supportClosed, false, Colors.Red);
            }
        }

        private void AddPathsToCanvas(PathsD paths, bool isOpen, Color color)
        {
            for (int i = 0; i < paths.Count; i++)
            {
                AddPathToCanvas(paths[i], isOpen, color);
            }
        }

        private void AddPathToCanvas(PathD path, bool isOpen, Color color)
        {
            double line_thickness = nozzleTickness;

            if (path.Count() <= 1)
                return;

            for (int point_index = 1; point_index < path.Count(); ++point_index)
            {
                Line line = new Line {
                    X1 = path[point_index - 1].x,
                    X2 = path[point_index].x,
                    Y1 = path[point_index - 1].y,
                    Y2 = path[point_index].y
                };

                line.Stroke = new SolidColorBrush(color);
                line.StrokeThickness = line_thickness;
                Canvas.Children.Add(line);
            }

            if (!isOpen)
            {
                // close path
                Line closing_line = new Line {
                    X1 = path[0].x,
                    X2 = path[path.Count - 1].x,
                    Y1 = path[0].y,
                    Y2 = path[path.Count - 1].y
                };

                closing_line.Stroke = new SolidColorBrush(color);
                closing_line.StrokeThickness = line_thickness;
                Canvas.Children.Add(closing_line);
            }
        }

        private void UpdateVars()
        {
            sliceHeight = Convert.ToDouble(TextBox_SliceHeight.Text, new System.Globalization.CultureInfo("en-US"));
            m_sliceGenerator.SetSliceHeight(sliceHeight);

            filamentTemp = Convert.ToInt32(TextBox_PrintTemperature.Text, new System.Globalization.CultureInfo("en-US"));
            bedTemp = Convert.ToInt32(TextBox_BedTemperature.Text, new System.Globalization.CultureInfo("en-US"));
            printSpeed = Convert.ToInt32(TextBox_PrintSpeed.Text, new System.Globalization.CultureInfo("en-US"));
            moveSpeed = Convert.ToInt32(TextBox_MoveSpeed.Text, new System.Globalization.CultureInfo("en-US"));
            m_gCodeGenerator.UpdateVars(filamentTemp, bedTemp, printSpeed, moveSpeed);

            Console.WriteLine("Slice Height: " + sliceHeight);
            Console.WriteLine("filament temp: " + filamentTemp);
            Console.WriteLine("Bed temp: " + bedTemp);
            Console.WriteLine("Print speed: " + printSpeed);
            Console.WriteLine("Move speed: " + moveSpeed);
        }

        private void UpdateSliceCount()
        {
            Rect3D bounds = m_model.GetBounds();
            sliceCount = (int)Math.Ceiling(bounds.SizeZ / sliceHeight);
            m_sliceGenerator.SetSliceCount(sliceCount);

            Label_SliceCount.Content = "Slice count: " + sliceCount;
        }

        private void UpdateSliderProperties()
        {
            Slider_Index.Minimum = 0;
            Slider_Index.Maximum = sliceCount - 1;
        }  

        private void ResetCheckboxSupport()
        {
            CheckBox_Support.IsChecked = true;
        }
        private void InitialCanvasProperties()
        {
            Label_Scale.Content = "Scale: " + Math.Round(canvasScale, 2);

            // Initial TransformMatrix for canavs
            Matrix mat = new Matrix(7.4, 0, 0, 7.4, -72, 175);
            MatrixTransform transMat = new MatrixTransform();
            transMat.Matrix = mat;
            Canvas.RenderTransform = transMat;

        }
    }
}
