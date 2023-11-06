using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

// Clipper
using Clipper2Lib;

namespace CF_slicer
{
    public class GCodeGenerator
    {
        private List<string> g_code;
        private PointD head_pos;
        private PointD last_head_pos;
        private double extrusion_pos;

        private int filamentTemp;
        private int bedTemp;
        private int printSpeed;
        private int moveSpeed;

        public GCodeGenerator()
        {
            g_code = new List<string>();
            head_pos = new PointD(0, 0);
            last_head_pos = new PointD(0, 0);
        }

        public void WriteToFile(string filePath)
        {
            File.WriteAllLines(filePath, g_code);
        }

        private void reset()
        {
            g_code.Clear();
            UpdateHeadPos(new PointD(0, 0));
            extrusion_pos = 0;
        }

        public void UpdateVars(int filamentTemp,int bedTemp,int printSpeed,int moveSpeed)
        {
            this.filamentTemp = filamentTemp;
            this.bedTemp = bedTemp;
            this.printSpeed = printSpeed;
            this.moveSpeed = moveSpeed;
        }

        public void GenerateGCode(List<List<PathData>> pathsPerSlice, double layerHeight, double flowModifier, double nozzleDiameter)
        {
            // Reset gcode
            reset();

            AddBeginPart();

            const double offsetToMiddle = 100;

            Console.WriteLine("G-CODE| ---------------------------Creating g-code for slices: " + pathsPerSlice.Count);

            // mark layer count 
            g_code.Add(";LAYER_COUNT:" + pathsPerSlice.Count);
            // Go over all slices
            // -> 1 slice for testing (needs to be m_pathsPerSlice.Count)
            for (int slice_index = 0; slice_index < pathsPerSlice.Count; slice_index++)
            {
                List<PathData> paths= pathsPerSlice[slice_index];
                
                Console.WriteLine("G-CODE| -----------Starting slice " + (slice_index + 1) + "/" + paths.Count);

                // mark layer start
                g_code.Add(";LAYER:" + slice_index);

                // Go over all polygons of slices
                for (int path_index = 0; path_index < paths.Count; path_index++)
                {
                    PathD path = paths[path_index].path;

                    Console.WriteLine("G-CODE| -------Starting path " + (path_index + 1) + "/" + paths.Count);

                    AddPathToCode(path, paths[path_index].isOpen, offsetToMiddle, layerHeight, slice_index, flowModifier, nozzleDiameter);
                }

                if (slice_index == 0)
                {
                    // turn fan on after first layer printed
                    g_code.Add("M106 ;turn fan on");
                }
            }

            AddEndPart();
        }

        private int WrapIndex(int i, int i_max)
        {
            return ((i % i_max) + i_max) % i_max;
        }

        private int CalcPathDirection(PathD path, bool isOpen, int first_point_index)
        {
            // open paths can't loop
            if (isOpen)
            {
                if (first_point_index == 0)
                {
                    return 1;
                }
                return -1;
            }

            PointD next = path[WrapIndex(first_point_index + 1, path.Count)];
            PointD current = path[first_point_index];
            PointD prev = path[WrapIndex(first_point_index - 1, path.Count)];

            Vector last_dir = new Vector(head_pos.x - last_head_pos.x, head_pos.y - last_head_pos.y);

            Vector next_dir = new Vector(next.x - current.x, next.y - current.y);
            Vector prev_dir = new Vector(prev.x - current.x, prev.y - current.y);

            double next_angle = Vector.AngleBetween(last_dir, next_dir);
            double prev_angle = Vector.AngleBetween(last_dir, prev_dir);

            if (Math.Abs(next_angle) < Math.Abs(prev_angle))
            {
                return 1;
            }
            return -1;
        }


        private void AddPathToCode(PathD path, bool openPath, double offsetToMiddle, double layerHeight, int slice_index, double flowModifier, double nozzleDiameter)
        {
            if (path.Count <= 0)
            {
                return;
            }

            // find closest point to head
            int first_point_index = SearchClosestPointIndexToHead(path, openPath);

            // calc path direction
            int path_direction = CalcPathDirection(path, openPath, first_point_index);

            Console.WriteLine("G-CODE| Moving to point " + (first_point_index) + "/" + path.Count);
            StartExtrusion(path[first_point_index], offsetToMiddle, layerHeight, slice_index);
            {
                int point_index = WrapIndex(first_point_index + path_direction, path.Count);
                // Go over all other points of all polygons
                while (point_index != first_point_index)
                {
                    //Console.WriteLine("G-CODE| Extruding to point " + (point_index + 1) + "/" + path.Count);
                    ExtrudeToPoint(path[point_index], offsetToMiddle, layerHeight, flowModifier, nozzleDiameter);
                    point_index = WrapIndex(point_index + path_direction, path.Count);
                }

                // extrude back towards first point if closed path
                if (!openPath)
                {
                    //Console.WriteLine("G-CODE| Extruding to point " + (first_point_index) + "/" + path.Count);

                    Vector p1 = new Vector(path[point_index].x, path[point_index].y);
                    Vector p2 = new Vector(path[first_point_index].x, path[first_point_index].y);
                    Vector p_to_last = (p2 - p1);
                    if (p_to_last.Length > 0)
                    {
                        p_to_last.Normalize();
                    }
                    Vector last_point_vec = p2 - p_to_last * nozzleDiameter;
                    PointD last_point = new PointD(last_point_vec.X, last_point_vec.Y);
                    ExtrudeToPoint(last_point, offsetToMiddle, layerHeight, flowModifier, nozzleDiameter);
                }
            }
            StopExtrusion();
        }
        private int SearchClosestPointIndexToHead(PathD path, bool isOpen)
        {
            if (isOpen)
            {
                // only look at first and last point
                int last_index = path.Count - 1;
                if (DistanceBetweenPoints(head_pos, path[0]) < DistanceBetweenPoints(head_pos, path[last_index]))
                {
                    return 0;
                }
                return last_index;
            } else
            {
                int closest = 0;
                double closest_dist = Double.MaxValue;
                for (int point_index = 0; point_index < path.Count; ++point_index)
                {
                    double dist_to_head = DistanceBetweenPoints(head_pos, path[point_index]);
                    if (closest_dist > dist_to_head)
                    {
                        closest = point_index;
                        closest_dist = dist_to_head;
                    }
                }
                return closest;
            }
        }

        private void StartExtrusion(PointD startPos, double offsetToMiddle, double layerHeight, int slice_index)
        {
            //retract filament to avoid oozing
            double move_dist = DistanceBetweenPoints(head_pos, startPos);
            double retraction = -move_dist / 10.0;
            extrusion_pos = extrusion_pos  + retraction;
            g_code.Add("G1 F" + moveSpeed +" E" + extrusion_pos.ToString().Replace(',', '.') + " ;retract filament to avoid oozing");

            // move head to first point of path without extrusion
            MoveToPoint(startPos, offsetToMiddle, layerHeight, slice_index);

            // move extrusion position to 0 to start extrusion
            extrusion_pos = 0;
            g_code.Add("G1 F" + printSpeed +" E" + extrusion_pos.ToString().Replace(',', '.') + " ;start filament extrusion");
        }
        private void StopExtrusion()
        {
            // reset extrusion pos to 0
            g_code.Add("G92 E0 ;reset extruder");
            extrusion_pos = 0;
        }

        // move head to point without extruding
        private void MoveToPoint(PointD point, double offsetToMiddle, double layerHeight, int slice_index)
        {
            string g_code_line =
                "G0 F" + moveSpeed +
                " X" + Math.Round(point.x + offsetToMiddle, 3) +
                " Y" + Math.Round(point.y + offsetToMiddle, 3) +
                " Z" + Math.Round((1 + slice_index) * layerHeight, 5);

            // replace ',' in doubles to '.'
            g_code.Add(g_code_line.Replace(',', '.'));

            // update new head positon
            UpdateHeadPos(point);
        }

        // move head to point with extrusion
        private void ExtrudeToPoint(PointD point, double offsetToMiddle, double layerHeight, double flowModifier, double nozzleDiameter)
        {
            double E = CalculateExtrude(layerHeight, flowModifier, nozzleDiameter, head_pos, point);

            // move to point with extrusion
            string g_code_line = 
                "G1" + 
                " X" + Math.Round(point.x + offsetToMiddle, 3) + 
                " Y" + Math.Round(point.y + offsetToMiddle, 3) + 
                " E" + Math.Round(extrusion_pos + E, 5);
            
            // replace ',' in doubles to '.'
            g_code.Add(g_code_line.Replace(',', '.'));

            // update new head positon
            UpdateHeadPos(point);

            // update extrusion position
            extrusion_pos += E;
        }

        private void UpdateHeadPos(PointD new_pos)
        {
            last_head_pos = head_pos;
            head_pos = new_pos;
        }

        private double CalculateExtrude(double layerHeight, double flowModifier, double nozzleDiameter, PointD p1, PointD p2)
        {
            double filament_diameter = 1.75;
            double dist = DistanceBetweenPoints(p1, p2) ;
            //Console.WriteLine(layerHeight + " " + flowModifier + " " + nozzleDiameter + " " +  dist);

            double filament_area = Math.PI * Math.Pow(filament_diameter / 2, 2);
            double E = (flowModifier*layerHeight * nozzleDiameter * dist) / filament_area;
            
            return E;
        }

        private double DistanceBetweenPoints(PointD p1, PointD p2)
        {
            return Math.Sqrt(Math.Pow((p2.x - p1.x), 2) + Math.Pow(p2.y - p1.y, 2));
        }

        private void AddBeginPart()
        {
            g_code.Add(";------------------ Initialization phase ------------------");
            g_code.Add("M140 S" + bedTemp + " ;set bed temperature");
            g_code.Add("M104 S" + filamentTemp +" ;set nozzle temperature");
            g_code.Add("M190 S" + bedTemp + " ;wait for bed temperature");
            g_code.Add("M109 S" + filamentTemp + " ;wait for nozzle temperature");
            g_code.Add("M82 ;absolute extrusion mode");
            g_code.Add("G28 ;home all axes");
            g_code.Add("G92 E0 ;reset extruder");
            g_code.Add("G1 Z2.0 F3000 ;move Z axis up little to prevent scratching of heat bed");
            g_code.Add("G1 X0.1 Y20 Z0.3 F5000 ;move to start position");
            g_code.Add("G1 X0.1 Y200 Z0.3 F1500 E15 ;draw first line");
            g_code.Add("G1 X0.4 Y200 Z0.3 F5000 ;move to side a little");
            g_code.Add("G1 X0.4 Y20 Z0.3 F1500 E30 ;draw second line");
            g_code.Add("G92 E0 ;reset extruder");
            g_code.Add("G1 Z2.0 F3000 ;move Z axis up little to prevent scratching of heat bed");
            g_code.Add("G92 E0 ;reset extruder");
            g_code.Add("G1 F2400 E-5 ;retract filament to avoid oozing");
            g_code.Add("M107 ;fan off for first layer");
            extrusion_pos = -5;
        }


        private void AddEndPart()
        {
            g_code.Add(";------------------ Reset phase ------------------");
            g_code.Add("M140 S0 ;set bed temperature");
            g_code.Add("M107 ;fan off");
            g_code.Add("M220 S100 ;reset speed factor override percentage to default (100%)");
            g_code.Add("M221 S100 ;reset extrude factor override percentage to default (100%)");
            g_code.Add("G91 ;set coordinates to relative");
            g_code.Add("G1 F1800 E-3 ;retract filament to prevent oozing");
            g_code.Add("G1 F3000 Z20 ;move Z axis up to allow filament to ooze freely");
            g_code.Add("G90 ;set coordinates to absolute");
            g_code.Add("G1 X0 Y235 F1000 ;move heat bed to front for easy print removal");
            g_code.Add("M107 ;fan off");
            g_code.Add("M84 ;disable stepper motors");
            g_code.Add("M82 ;absolute extrusion mode");
            g_code.Add("M104 S0 ;set extruder temperature");
        }
    }
}
