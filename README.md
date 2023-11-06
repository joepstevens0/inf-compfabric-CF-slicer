# CF-slicer
Slicer project for Computational Fabrication course

## Roadmap
- [x] Load and render a 3D STL model
- [X] Make a basic UI to control the height of a slicing plane with respect to the 3D model
- [X] Write an algorithm to slice the 3D model at the position of the slicing plane. Visualize the perimeter
of this cross-section in a 2D canvas.
- [X] Create polygons from all lines (Clipper requires polygons for its algorithms)
- [X] Erode the perimeter with half the nozzle thickness (otherwise, the print will be too large).
- [X] Generate g-code for printing the perimeter of a single slice and try 3D printing.
- [X] Extend data structure to support holes in manifold 3D objects.
- [X] Write code for supporting a variable number of shells.
- [X] Generate a basic sparse rectangular infill structure.
- [X] Extend g-code generation and 3D print a simple object that does not require support structures.
- [X] Calculate regions + generate toolpaths (+ g-code) for floors and roofs.
- [X] Try 3D printing a closed object (roofs + floors) that does not require support.
- [X] Features + algorithms for basic (straight) support structure generation.
- [X] Implement all other minimal requirements.
