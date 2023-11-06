using System;
using System.Collections.Generic;
using System.Linq;

using System.Windows.Media;
using System.Windows.Media.Media3D;

// Clipper
using Clipper2Lib;

namespace CF_slicer
{
    struct Triangle3D
    {
        public Triangle3D(Point3D p1_, Point3D p2_, Point3D p3_)
        {
            p1 = p1_;
            p2 = p2_;
            p3 = p3_;
        }

        public Point3D p1;
        public Point3D p2;
        public Point3D p3;
    }

    public struct PathData
    {
        public PathD path;
        public bool isOpen;

        public PathData(PathD path, bool isOpen)
        {
            this.path = path;
            this.isOpen = isOpen;
        }
    }

    class SliceInfo
    {
        public SliceInfo()
        {
            input_slice = new PathsD();
            holed = new PathsD();
            first_shell = new PathsD();
            last_computed_shell = new PathsD();
            last_computed_result = new PathsD();
            open_paths = new PathsD();

            // For drawing purposes only
            contour = new PathsD();
            shells = new PathsD();
            infill = new PathsD();
            floorsAndRoofs = new PathsD();
            supportClosed = new PathsD();
            supportOpen = new PathsD();
        }

        public PathsD input_slice;
        public PathsD holed;
        public PathsD first_shell;
        public PathsD last_computed_shell;
        public PathsD last_computed_result;
        public PathsD open_paths;

        // For drawing purposes only
        public PathsD contour;
        public PathsD shells;
        public PathsD infill;
        public PathsD floorsAndRoofs;
        public PathsD supportClosed;
        public PathsD supportOpen;
    }
    class SliceGenerator
    {
        // Input
        private double m_minZ;
        private double m_minX;
        private double m_minY;
        private double m_sizeX;
        private double m_sizeY;

        private double m_sliceHeight;
        private double m_sliceCount;
        private double m_nozzleThickness;
        private int m_totalShells;
        private List<Triangle3D> m_triangles;


        // Output
        private List<SliceInfo> m_slices;

        public SliceGenerator() { }

        public SliceGenerator(double meshMinZ, double meshMinX, double meshMinY, double meshSizeX, double meshSizeY, Point3DCollection vertices, Int32Collection indices)
        {
            Initialize(meshMinZ, meshMinX, meshMinY, meshSizeX, meshSizeY, vertices, indices);
        }

        public void Initialize(double meshMinZ, double meshMinX, double meshMinY, double meshSizeX, double meshSizeY, Point3DCollection vertices, Int32Collection indices)
        {
            m_minZ = meshMinZ;
            m_minX = meshMinX;
            m_minY = meshMinY;
            m_sizeX = meshSizeX;
            m_sizeY = meshSizeY;

            m_triangles = GetTriangles(vertices, indices);

            Console.WriteLine("Min Z: {0}", m_minZ);

            // Empty
            m_slices = new List<SliceInfo>();
        }

        public int TotalSlices()
        {
            return m_slices.Count;
        }

        public void SetSliceHeight(double sliceHeight)
        {
            m_sliceHeight = sliceHeight;
        }
        public void SetSliceCount(int sliceCount)
        {
            m_sliceCount = sliceCount;
        }
        public void SetNozzleThickness(double nozzleThickness)
        {
            m_nozzleThickness = nozzleThickness;
        }
        public void SetTotalShells(int totalShells)
        {
            m_totalShells = totalShells;
        }

        public void GenerateSlices()
        {
            m_slices.Clear();
            for (int i = 0; i < m_sliceCount; i++)
            {
                double planeZ = CalcPlaneZ(i);
                SliceInfo info = new SliceInfo();
                info.input_slice = CreateSlice(planeZ);
                m_slices.Add(info);
            }
        }

        public void ReprocessSlices(bool add_support)
        {
            // clear slices
            for (int i = 0; i < m_slices.Count; i++)
            {
                m_slices[i].holed.Clear();
                m_slices[i].first_shell.Clear();
                m_slices[i].last_computed_shell.Clear();
                m_slices[i].last_computed_result.Clear();
                m_slices[i].open_paths.Clear();

                m_slices[i].contour.Clear();
                m_slices[i].shells.Clear();
                m_slices[i].infill.Clear();
                m_slices[i].floorsAndRoofs.Clear();
                m_slices[i].supportOpen.Clear();
                m_slices[i].supportClosed.Clear();
            }

            CalcSolidAndHoles();

            ApplyNozzleTickness();
            ApplyTotalShells();
            //RemoveOverlap();
            ApplyInfill();

            if (add_support)
                ApplySupport();
            RemoveShortPaths();
        }

        private void RemoveShortPaths()
        {
            // remove paths that are to short
            double EPSILON = 0.3;
            for (int i = 0; i < m_slices.Count; i++)
            {
                SliceInfo slice = m_slices[i];
                for (int pathIndex = 0; pathIndex < slice.open_paths.Count; ++pathIndex)
                {
                    if (PathLength(slice.open_paths[pathIndex]) < EPSILON)
                    {
                        slice.open_paths.RemoveAt(pathIndex);
                        pathIndex -= 1;
                    }
                }

                for (int pathIndex = 0; pathIndex < slice.last_computed_result.Count; ++pathIndex)
                {
                    if (PathLength(slice.last_computed_result[pathIndex]) < EPSILON)
                    {
                        slice.last_computed_result.RemoveAt(pathIndex);
                        pathIndex -= 1;
                    }
                }
            }
        }

        private double PathLength(PathD path)
        {
            double r = 0;
            for (int i = 1; i < path.Count; ++i)
            {
                double dX = path[i - 1].x - path[i].x;
                double dY = path[i - 1].y - path[i].y;
                double distance = Math.Sqrt(dX * dX + dY * dY);
                r += distance;
            }
            return r;
        }

        public void ApplyNozzleTickness()
        {
            for (int sliceIndex = 0; sliceIndex < m_slices.Count; ++sliceIndex)
            {
                SliceInfo slice = m_slices[sliceIndex];
                PathsD new_paths = new PathsD();

                for (int i = 0; i < slice.last_computed_result.Count; ++i)
                {
                    if (Clipper.IsPositive(slice.last_computed_result[i]))
                    {
                        // Apply erosion to solids
                        new_paths.AddRange(InflatePath(slice.last_computed_result[i], -m_nozzleThickness / 2.0));
                    }
                    else
                    {
                        // Apply inflation to holes
                        new_paths.AddRange(InflatePath(slice.last_computed_result[i], m_nozzleThickness / 2.0));

                    }
                }
                slice.last_computed_result = new_paths;
                slice.contour = new_paths;
            }
        }

        public void ApplyTotalShells()
        {
            for (int sliceIndex = 0; sliceIndex < m_slices.Count; ++sliceIndex)
            {
                SliceInfo slice = m_slices[sliceIndex];
                slice.first_shell = new PathsD(slice.last_computed_result);
                PathsD new_paths = new PathsD(slice.last_computed_result);

                slice.last_computed_shell = slice.first_shell;
                for (int i = 1; i < m_totalShells; ++i)
                {
                    PathsD shell = InflatePaths(slice.first_shell, m_nozzleThickness * i);
                    shell = Clipper.Xor(shell, new PathsD(), Clipper2Lib.FillRule.Positive, 8);
                    new_paths.AddRange(shell);
                    slice.last_computed_shell = shell;
                }
                slice.last_computed_result = new_paths;
                slice.shells = new_paths;
            }
        }

        public PathsD InflatePaths(PathsD paths, double inflateAmount)
        {
            PathsD inflatedPaths = new PathsD();
            for (int polyIndex = 0; polyIndex < paths.Count; ++polyIndex)
            {
                if (Clipper.IsPositive(paths[polyIndex]))
                {
                    // Apply inflation to solids
                    inflatedPaths.AddRange(InflatePath(paths[polyIndex], -inflateAmount));
                }
                else
                {
                    // Apply inflation to holes
                    inflatedPaths.AddRange(InflatePath(paths[polyIndex], inflateAmount));
                }
            }
            return inflatedPaths;
        }

        // returns closed paths of floors
        private List<PathsD> CalcFloors()
        {
            const int BOTTOM_FLOOR_LAYERS = 2;

            List<PathsD> floorsPerSlice = new List<PathsD>();

            for (int i = 0; i < m_slices.Count; ++i)
            {
                if (i < BOTTOM_FLOOR_LAYERS)
                {
                    // first n layers are always floors
                    floorsPerSlice.Add(m_slices[i].last_computed_shell);
                } else
                {
                    // intersect last n floors
                    PathsD lastFloors = m_slices[i - BOTTOM_FLOOR_LAYERS].last_computed_shell;
                    for (int slice = i - BOTTOM_FLOOR_LAYERS + 1; slice < i; ++slice)
                    {
                        lastFloors = Clipper.Intersect(lastFloors, m_slices[slice].last_computed_shell, Clipper2Lib.FillRule.EvenOdd);
                    }
                    PathsD sliceFloor = Clipper.Difference(m_slices[i].last_computed_shell, lastFloors, Clipper2Lib.FillRule.EvenOdd);
                    floorsPerSlice.Add(sliceFloor);
                }
            }

            return floorsPerSlice;
        }

        // returns closed paths of roofs
        private List<PathsD> CalcRoofs()
        {
            const int TOP_ROOF_LAYERS = 2;

            List<PathsD> roofsPerSlice = new List<PathsD>();

            for (int i = 0; i < m_slices.Count; ++i)
            {
                if (i + TOP_ROOF_LAYERS >= m_slices.Count)
                {
                    // top n layers are always floors
                    roofsPerSlice.Add(m_slices[i].last_computed_shell);
                } else
                {
                    // intersect last n floors
                    PathsD nextRoofs = m_slices[i + 1].last_computed_shell;
                    for (int slice = i + 2; slice <= i + TOP_ROOF_LAYERS; ++slice)
                    {
                        nextRoofs = Clipper.Intersect(nextRoofs, m_slices[slice].last_computed_shell, Clipper2Lib.FillRule.EvenOdd);
                    }
                    PathsD sliceFloor = Clipper.Difference(m_slices[i].last_computed_shell, nextRoofs, Clipper2Lib.FillRule.EvenOdd);
                    roofsPerSlice.Add(sliceFloor);
                }
            }

            return roofsPerSlice;
        }

        public void ApplyInfill()
        {

            PathsD infill_hor = new PathsD();
            PathsD infill_vert = new PathsD();

            PathsD floor_and_roof_hor = new PathsD();
            PathsD floor_and_roof_vert = new PathsD();

            const double OFFSET = 10;
            double STEP = 2;
            
            double MIN_X = m_minX - OFFSET;
            double MAX_X = m_minX + m_sizeX + OFFSET;
            double MIN_Y = m_minY - OFFSET;
            double MAX_Y = m_minY + m_sizeY + OFFSET;

            // Horizontal lines floor and roof
            for (double x = MIN_X; x < MAX_X; x += m_nozzleThickness)
            {
                PathD path_hor = new PathD {
                    new PointD(x, MIN_Y),
                    new PointD(x, MAX_Y)
                };
                floor_and_roof_hor.Add(path_hor);
            }

            // Vertical lines roof and floor
            for (double y = MIN_Y; y < MAX_Y; y += m_nozzleThickness)
            {
                PathD path_vert = new PathD {
                    new PointD(MIN_X, y),
                    new PointD(MAX_X, y)
                };
                floor_and_roof_vert.Add(path_vert);
            }

            // Horizontal lines infill
            for (double x = MIN_X; x < MAX_X; x += STEP)
            {
                PathD path_hor = new PathD {
                    new PointD(x, MIN_Y),
                    new PointD(x, MAX_Y)
                };
                infill_hor.Add(path_hor);
            }

            // Vertical lines infill
            for (double y = MIN_Y; y < MAX_Y; y += STEP)
            {
                PathD path_vert = new PathD {
                    new PointD(MIN_X, y),
                    new PointD(MAX_X, y)
                };
                infill_vert.Add(path_vert);
            }

            List<PathsD> floorsPerSlice = CalcFloors();
            List<PathsD> roofsPerSlice = CalcRoofs();

            for (int i = 0; i < m_slices.Count; ++i)
            {
                PathsD floorsAndRoofs = Clipper.Union(floorsPerSlice[i], roofsPerSlice[i], Clipper2Lib.FillRule.EvenOdd);

                // add floors and roofs
                {
                    ClipperD clipper = new ClipperD(8);
                    clipper.AddPaths(floorsAndRoofs, PathType.Clip);
                    if (i % 2 == 1)
                    {
                        clipper.AddPaths(floor_and_roof_hor, PathType.Subject, true);
                    } else
                    {
                        clipper.AddPaths(floor_and_roof_vert, PathType.Subject, true);
                    }
                    PathsD solutionOpen = new PathsD();
                    PathsD solutionClosed = new PathsD();
                    clipper.Execute(ClipType.Intersection, Clipper2Lib.FillRule.EvenOdd, solutionClosed, solutionOpen);
                    m_slices[i].open_paths.AddRange(solutionOpen);
                    m_slices[i].last_computed_result.AddRange(solutionClosed);

                    m_slices[i].floorsAndRoofs.AddRange(solutionOpen);
                }
                // add infill
                {
                    // remove floors and roofs from shell
                    PathsD shell = Clipper.Difference(m_slices[i].last_computed_shell, floorsAndRoofs, Clipper2Lib.FillRule.EvenOdd);

                    ClipperD clipper = new ClipperD(8);
                    clipper.AddPaths(shell, PathType.Clip);
                    clipper.AddPaths(infill_hor, PathType.Subject, true);
                    clipper.AddPaths(infill_vert, PathType.Subject, true);
                    PathsD solutionOpen = new PathsD();
                    PathsD solutionClosed = new PathsD();
                    clipper.Execute(ClipType.Intersection, Clipper2Lib.FillRule.EvenOdd, solutionClosed, solutionOpen);
                    m_slices[i].open_paths.AddRange(solutionOpen);
                    m_slices[i].last_computed_result.AddRange(solutionClosed);

                    m_slices[i].infill.AddRange(solutionOpen);
                }
            }
        }

        public void ApplySupport()
        {
            PathsD support_hor = new PathsD();
            PathsD support_vert = new PathsD();
            const double OFFSET = 10;
            double STEP = 8;
            double MIN_X = m_minX - OFFSET;
            double MAX_X = m_minX + m_sizeX + OFFSET;
            double MIN_Y = m_minY - OFFSET;
            double MAX_Y = m_minY + m_sizeY + OFFSET;
            // Horizontal lines infill
            for (double x = MIN_X; x < MAX_X; x += STEP)
            {
                PathD path_hor = new PathD {
                    new PointD(x, MIN_Y),
                    new PointD(x, MAX_Y)
                };
                support_hor.Add(path_hor);
            }
            // Vertical lines infill
            for (double y = MIN_Y; y < MAX_Y; y += STEP)
            {
                PathD path_vert = new PathD {
                    new PointD(MIN_X, y),
                    new PointD(MAX_X, y)
                };
                support_vert.Add(path_vert);
            }

            PathsD lastLayer = m_slices[m_slices.Count - 1].first_shell;
            for (int i = m_slices.Count - 2; i >= 0; --i)
            {
                PathsD firstShellInflated = Clipper.InflatePaths(m_slices[i].first_shell, m_nozzleThickness/2, JoinType.Round, EndType.Polygon);
                PathsD supportArea = Clipper.Difference(lastLayer, firstShellInflated, Clipper2Lib.FillRule.EvenOdd);

                // add support infill
                {
                    //// remove last layer of support
                    //PathsD realSupport = Clipper.Difference(supportArea, m_slices[i + 1].first_shell, Clipper2Lib.FillRule.EvenOdd);

                    // connect support areas that are a nozzle diameter/2 apart
                    PathsD realSupport = Clipper.InflatePaths(supportArea, m_nozzleThickness / 2, JoinType.Round, EndType.Polygon);
                    realSupport = Clipper.Union(realSupport, Clipper2Lib.FillRule.Positive);
                    realSupport = Clipper.InflatePaths(realSupport, -m_nozzleThickness / 2, JoinType.Round, EndType.Polygon);
                    realSupport = Clipper.Difference(
                        Clipper.InflatePaths(realSupport, -m_nozzleThickness, JoinType.Round, EndType.Polygon)
                        , Clipper.InflatePaths(m_slices[i].first_shell, 2 * m_nozzleThickness
                        , JoinType.Round, EndType.Polygon), Clipper2Lib.FillRule.EvenOdd);


                    // intsect with support structure
                    ClipperD clipper = new ClipperD(8);
                    clipper.AddPaths(realSupport, PathType.Clip);
                    clipper.AddPaths(support_hor, PathType.Subject, true);
                    clipper.AddPaths(support_vert, PathType.Subject, true);
                    PathsD solutionOpen = new PathsD();
                    PathsD solutionClosed = new PathsD();
                    clipper.Execute(ClipType.Intersection, Clipper2Lib.FillRule.EvenOdd, solutionClosed, solutionOpen);
                    m_slices[i].open_paths.AddRange(solutionOpen);
                    m_slices[i].last_computed_result.AddRange(solutionClosed);
                    m_slices[i].last_computed_result.AddRange(realSupport);

                    m_slices[i].supportOpen.AddRange(solutionOpen);
                    m_slices[i].supportClosed.AddRange(solutionClosed);
                    m_slices[i].supportClosed.AddRange(realSupport);
                }

                // use last support area without shell inflation for calculations
                lastLayer = Clipper.Union(supportArea, m_slices[i].first_shell, Clipper2Lib.FillRule.EvenOdd);
            }
        }

        public void RemoveOverlap()
        {
            for (int sliceIndex = 0; sliceIndex < m_slices.Count; ++sliceIndex)
            {
                SliceInfo slice = m_slices[sliceIndex];
                slice.last_computed_result = Clipper.Union(slice.last_computed_result, Clipper2Lib.FillRule.EvenOdd);
            }
        }

       

        private PathsD InflatePath(PathD path, double amount)
        {
            return Clipper.InflatePaths(new PathsD{ path }, amount, JoinType.Round, EndType.Polygon);
        }


        public PathsD GetOriginalPath(int sliceIndex)
        {
            return m_slices[sliceIndex].input_slice;
        }
        public List<List<PathData>> GetPaths()
        {
            // Get all paths together (solids and holes)
            List<List<PathData>> allPaths = new List<List<PathData>>();
            for (int sliceIndex = 0; sliceIndex < m_slices.Count; sliceIndex++)
            {
                allPaths.Add(GetPathsBySlice(sliceIndex));
            }
            return allPaths;
        }

        public List<PathData> GetPathsBySlice(int sliceIndex)
        {
            List<PathData> paths = new List<PathData>();
            // add closed paths
            for (int pathIndex = 0; pathIndex < m_slices[sliceIndex].last_computed_result.Count; ++pathIndex)
            {
                paths.Add(new PathData(m_slices[sliceIndex].last_computed_result[pathIndex], false ));
            }
            // ad open paths
            for (int pathIndex = 0; pathIndex < m_slices[sliceIndex].open_paths.Count; ++pathIndex)
            {
                paths.Add(new PathData(m_slices[sliceIndex].open_paths[pathIndex], true));
            }
            return paths;
        }

        public SliceInfo GetSliceInfo(int sliceIndex)
        {
            return m_slices[sliceIndex];
        }


        private List<Triangle3D> GetTriangles(Point3DCollection vertices, Int32Collection indices)
        {
            List<Triangle3D> triangles = new List<Triangle3D>();

            for (int i = 0; i < indices.Count; i += 3)
            {
                Point3D p1 = vertices[indices[i]];
                Point3D p2 = vertices[indices[i + 1]];
                Point3D p3 = vertices[indices[i + 2]];

                Triangle3D triangle = new Triangle3D(p1, p2, p3);
                triangles.Add(triangle);
            }

            return triangles;
        }

        public double GetMinZ()
        {
            return m_minZ;
        }

        /// <summary>
        /// Calculate Z of intersection plane by index of slice. 
        /// (middle = best accuracy)
        /// </summary>
        /// <param name="i"> Index of slice </param>
        /// <returns></returns>
        public double CalcPlaneZ(int i)
        {
            const double doubleOffset = 0.000000001;
            return m_minZ + m_sliceHeight * i + (m_sliceHeight / 2) + doubleOffset;
        }

        private PathsD CreateSlice(double planeZ)
        {
            PathsD lines = new PathsD();

            // Check all triangles for intersection with plane
            for (int i = 0; i < m_triangles.Count; i++)
            {
                Triangle3D triangle = m_triangles[i];
                float zMax = (float)Math.Max(triangle.p1.Z, Math.Max(triangle.p2.Z, triangle.p3.Z));
                float zMin = (float)Math.Min(triangle.p1.Z, Math.Min(triangle.p2.Z, triangle.p3.Z));

                // Skip triangles that can't intersect with plane
                if (zMax < planeZ || zMin > planeZ)
                {
                    continue;
                }

                lines.Add(TrianglePlaneIntersection(planeZ, triangle));
            }

            return LinesToPaths(lines);
        }


        /// <summary>
        /// Check which 2/3 edges intersect with plane and calculate line between intersecting points
        /// </summary>
        private PathD TrianglePlaneIntersection(double planeZ, Triangle3D triangle)
        {
            List<PointD> linePoints = new List<PointD>();

            // Edge p1 - p2 
            if (((float)triangle.p1.Z > planeZ && (float)triangle.p2.Z < planeZ) ||
                ((float)triangle.p1.Z < planeZ && (float)triangle.p2.Z > planeZ))
            {
                linePoints.Add(EdgePlaneIntersection(planeZ, triangle.p1, triangle.p2));
            }
            // Edge p1 - p3
            if (((float)triangle.p1.Z > planeZ && (float)triangle.p3.Z < planeZ) ||
                ((float)triangle.p1.Z < planeZ && (float)triangle.p3.Z > planeZ))
            {
                linePoints.Add(EdgePlaneIntersection(planeZ, triangle.p1, triangle.p3));
            }

            // Edge p2 - p3 
            if (((float)triangle.p2.Z > planeZ && (float)triangle.p3.Z < planeZ) ||
                ((float)triangle.p2.Z < planeZ && (float)triangle.p3.Z > planeZ))
            {
                linePoints.Add(EdgePlaneIntersection(planeZ, triangle.p2, triangle.p3));
            }

            // Create line from points
            PathD line = CreateLine(linePoints[0], linePoints[1]);

            return line;
        }

        private PointD EdgePlaneIntersection(double planeZ, Point3D p1, Point3D p2)
        {
            PointD intersectingPoint = new PointD
            {
                x = p1.X + ((planeZ - p1.Z) * (p2.X - p1.X) / (p2.Z - p1.Z)),
                y = p1.Y + ((planeZ - p1.Z) * (p2.Y - p1.Y) / (p2.Z - p1.Z))
            };

            return intersectingPoint;
        }

        private PathD CreateLine(PointD p1, PointD p2)
        {
            PathD line = new PathD { };
            line.Add(p1);
            line.Add(p2);

            return line;
        }

        private PathsD LinesToPaths(PathsD lines)
        {
            PathsD result = new PathsD();
            Queue<PathD> unused_lines = new Queue<PathD>(lines);

            while (unused_lines.Count() > 0)
            {
                // use first line as new path start
                PathD current_path = unused_lines.Dequeue();

                int total_unused_needed_checking = unused_lines.Count();
                while (total_unused_needed_checking > 0)
                {
                    PathD line = unused_lines.Dequeue();

                    // Check start point of line
                    if (IsEpsilonDistance(current_path.Last(), line[0]))
                    {
                        // take average of the connected points
                        double x = (current_path.Last().x + line[0].x) / 2;
                        double y = (current_path.Last().y + line[0].y) / 2;
                        current_path[current_path.Count() - 1] = new PointD(x, y);

                        // add line to path
                        current_path.Add(line[1]);

                        // look at all other lines again
                        total_unused_needed_checking = unused_lines.Count();

                    }

                    // Check end point of line
                    else if (IsEpsilonDistance(current_path.Last(), line[1]))
                    {
                        // take average of the connected points
                        double x = (current_path.Last().x + line[1].x) / 2;
                        double y = (current_path.Last().y + line[1].y) / 2;
                        current_path[current_path.Count() - 1] = new PointD(x, y);

                        // add line to path
                        current_path.Add(line[0]);

                        // look at all other lines again
                        total_unused_needed_checking = unused_lines.Count();
                    }

                    else
                    {
                        // line does not fit, return it to list
                        unused_lines.Enqueue(line);
                        total_unused_needed_checking -= 1;
                    }
                }
                result.Add(current_path);
            }


            return result;
        }

        private bool IsEpsilonDistance(PointD p1, PointD p2)
        {
            const double LINE_EPSILON = 0.000001;

            double dX = p1.x - p2.x;
            double dY = p1.y - p2.y;
            double distance = Math.Sqrt(dX * dX + dY * dY);

            return distance < LINE_EPSILON;
        }

        public void CalcSolidAndHoles()
        {

            for (int sliceIndex = 0; sliceIndex < m_slices.Count; sliceIndex++)
            {
                //Console.WriteLine("==========================================================");
                //Console.WriteLine("Slice: " + sliceIndex);

                CalculateSolidAndHolesSlice(m_slices[sliceIndex]);
            }
            
        }

        private void CalculateSolidAndHolesSlice(SliceInfo output)
        {
            output.holed = Clipper.Xor(output.input_slice, new PathsD(), Clipper2Lib.FillRule.EvenOdd);
            output.last_computed_result = output.holed;
        }
    }
}
