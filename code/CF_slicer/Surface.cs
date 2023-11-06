using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace CF_slicer
{
    class Surface
    {
        private RectangleVisual3D m_surface;

        public Surface() { }
        public Surface(RectangleVisual3D surface, double minX, double minY, double sizeX, double sizeY, double Z)
        {
            Initialize(surface, minX, minY, sizeX, sizeY, Z);
        }

        public void Initialize(RectangleVisual3D surface, double Z, double minX, double minY, double sizeX, double sizeY)
        {
            m_surface = surface;

            double originX = minX + sizeX / 2;
            double originY = minY + sizeY / 2;

            // Calculate surface with model bounds
            m_surface.Origin = new Point3D(originX, originY, Z);
            m_surface.Length = sizeX;
            m_surface.Width = sizeY;
        }

        public void MoveSurface(double Z)
        {
            m_surface.Transform = new TranslateTransform3D(0, 0, Z);
        }
    }
}
