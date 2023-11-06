using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;


// Helix
using HelixToolkit.Wpf;

using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace CF_slicer
{
    class Model
    {
        private MeshGeometry3D mesh;

        public Model()
        {

        }

        public void LoadModel(string filePath)
        {
            // Load mesh
            StLReader reader = new HelixToolkit.Wpf.StLReader();
            Model3DGroup group = reader.Read(filePath);
            GeometryModel3D geometryModel = FindLargestModel(group);
            MeshGeometry3D meshGeometry3D = geometryModel.Geometry as MeshGeometry3D;
            mesh = meshGeometry3D;
        }

        public Rect3D GetBounds()
        {
            return mesh.Bounds;
        }

        public Int32Collection GetIndices()
        {
            return mesh.TriangleIndices;
        }

        public Point3DCollection GetVertices()
        {
            return mesh.Positions;
        }

        public Vector3DCollection GetNormals()
        {
            return mesh.Normals;
        }

        private GeometryModel3D FindLargestModel(Model3DGroup group)
        {
            if (group.Children.Count == 1)
            {
                return group.Children[0] as GeometryModel3D;
            }

            int maxCount = int.MinValue;
            GeometryModel3D maxModel = null;
            foreach (GeometryModel3D model in group.Children)
            {
                int count = ((MeshGeometry3D)model.Geometry).Positions.Count;
                if (maxCount < count)
                {
                    maxCount = count;
                    maxModel = model;
                }
            }
            return maxModel;

        }
    }
}
