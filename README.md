# Python, C++ and Go implementation of the Ray Casting Algorithm (PIP)
Author: David Salac <http://www.github.com/david-salac>

Python, C++ Go implementation of the Ray Casting algorithm for solving of 
the Point In the Polygon (PIP) problem. The Ray Casting algorithm represents
a simple method for determining if an arbitrary point is inside polygon
or outside. 

## Description
One of the common task in many fields (like GIS) is to determine if an
arbitrary point is located inside or outside (or on the edge) of given
polygon. There are many algorithms for dealing with this problem. One
of the simplest and most powerful is called Ray Casting Algorithm.

The principle of Ray Casting Algorithm is that when you draw a half-line
(or a ray) from given point to any direction then the rule is that if the
number of edge crossing of this ray is an odd number, point is located
inside, if it is an even number, point is outside. 

## Implementation
This implementation provides simple version of RCA enriched by the dealing
with border cases (point on the edge) separately. If the point is on the
edge it is determined to be an inner point of the polygon (using the
separate function).

## Software User Manual
Polygon must be defined as a series of vertices sorted clock-wise.

### Go version
Demo in the Go language:
```
import "polygon_pip"

// Some arbitrary polygon
vertices := []polygon_pip.Vertex{
    polygon_pip.Vertex{2, 4},
    polygon_pip.Vertex{3, 5},
    polygon_pip.Vertex{4, 5},
    polygon_pip.Vertex{5, 4},
    polygon_pip.Vertex{4, 3},
    polygon_pip.Vertex{5, 1},
    polygon_pip.Vertex{4, 1},
    polygon_pip.Vertex{4, 2},
    polygon_pip.Vertex{3, 4}}

// Create new polygon from vertices
if polygon, error := polygon_pip.NewPolygon(vertices); error != nil {
    // Print errors (if any)
    fmt.Println(error)
} else {
    // Test if the points on grid are inside polygon or outside    
    for x := 0.0; x < 6.0; x += 0.1 {
        for y := 0.0; y < 6.0; y += 0.1 {
            // Test if the point lies inside polygon (or on the edge)
            if polygon.PipRayCastingAlgorithm(x, y) {
                fmt.Println("Point (%v, %v) lies inside polygon.", x, y)
            } else {
                fmt.Println("Point (%v, %v) lies outside polygon.", x, y)
            }
        }
    }
}
```
The main function is the `PipRayCastingAlgorithm(x, y)`
this function returns true if the point with coordinates `(x, y)` is located
inside of the polygon and false if outside.

Polygon can be created using `NewPolygon(vertices)` function that accepts
an array of indices as the parameter.

## Python version
Version in Python (in version above or equal to 3.6):
```
import polygon as plg
import matplotlib.pyplot as plt

# Create list of vertices
vertices = [plg.Vertex(2, 4),
            plg.Vertex(3, 5),
            plg.Vertex(4, 5),
            plg.Vertex(4.5, 3.5)]
# Create new polygon
polygon = plg.Polygon(vertices)

# Define the grid
x = [v/10 for v in range(0, 60)]
y = [v/10 for v in range(0, 60)]
x_in = []
y_in = []
x_out = []
y_out = []

# Analyse points on the grid
for i in range(len(x)):
    for j in range(len(y)):
        # Call the Ray Casting Algorithm implementation
        if polygon.pip_ray_casting_algorithm(x[i], y[j]):
            # Append the point to the set if inside polygon
            x_in += [x[i]]
            y_in += [y[j]]
        else:
            # Append the point to the set if outside polygon
            x_out += [x[i]]
            y_out += [y[j]]

# Plotting of the results (just demo using matplotlib.pyplot)
edges_X = []
edges_Y = []
for edge in polygon.edges:
    edges_X.append(edge.x_start)
    edges_X.append(edge.x_end)
    edges_Y.append(edge.y_start)
    edges_Y.append(edge.y_end)

plt.plot(edges_X, edges_Y)      # Plot the edges
plt.plot(x_in, y_in, 'ro')      # Plot points inside polygon
plt.plot(x_out, y_out, 'gx')    # Plot points outside polygon
plt.title('Points inside/outside of the polygon')
plt.show()
```
The main function is the `Polygon.pip_ray_casting_algorithm(x, y)`
this function returns true if the point with coordinates `(x, y)` is located
inside of the polygon and false if outside.

Polygon can be created using constructor that accepts an array of indices
as the parameter.

## C++ version
Version in C++ (version C++ 17):
```
#include "Polygon.hpp"

// Create list of vertices
std::vector<std::tuple<double, double>> vertices = std::vector<std::tuple<double, double>> {
        std::make_tuple(2.0, 4.0),
        std::make_tuple(3.0, 5.0),
        std::make_tuple(4.0, 5.0),
        std::make_tuple(5.0, 4.0),
        std::make_tuple(4.0, 3.0),
        std::make_tuple(5.0, 1.0),
        std::make_tuple(4.0, 1.0),
        std::make_tuple(4.0, 2.0),
        std::make_tuple(3.0, 4.0)
    };

// Create new polygon
Polygon polygon(vertices);

// Create vectors of coordinates of points inside/outside the polygon
std::vector<std::tuple<double, double>> pointsIn;
std::vector<std::tuple<double, double>> pointsOut;
for (double x = 0.0; x < 6.0; x += 0.1) {
    for (double y = 0.0; y < 6.0; y += 0.1) {
        // Test if the point lies inside polygon (or on the edge)
        if (polygon.PipRayCastingAlgorithm(x, y)) {
            pointsIn.push_back(std::make_tuple(x, y));
        } else {
            // If not, append to the another list
            pointsOut.push_back(std::make_tuple(x, y));
        }
    }
}
```
The main function is the `Polygon::PipRayCastingAlgorithm(x, y)`
this function returns true if the point with coordinates `(x, y)` is located
inside of the polygon and false if outside.

Polygon can be created using constructor that accepts an array of vertices
(defined as the tuple of `x, y` coordinates) as the parameter.
