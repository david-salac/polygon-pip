package polygon_pip

import (
    "errors"
    "fmt"
    "math"
    "strings"
)


// Do the IEEE double precision float (64 bit) delta comparison (with delta = 0.0001)
func DeltaCompare(a, b float64) bool {
    // Delta used for comparisons
    DELTA := 0.0001
    // Do comparison and return results
    return math.Abs(a - b) < DELTA
}


// Vertex (point) of the polygon.
type Vertex struct {
    CoordX  float64     // Horizontal position of the point.
    CoordY  float64     // Vertical position of the point.
}


// Edge (line) of the polygon. All Edges has to be added following clockwise logic.
type Edge struct {
    CoordXStart         float64 // Horizontal coordinates of starting vertex
    CoordYStart         float64 // Vertical coordinates of starting vertex
    CoordXEnd           float64 // Horizontal coordinates of ending vertex
    CoordYEnd           float64 // Vertical coordinates of ending vertex
    Singular            bool    // If true, edge is either horizontal or vertical line
    SingularByX         bool    // If singular and true, edge is horizontal line, vertical otherwise
    LinearCoefficient   float64 // If non-singular, linear coefficient of the line defining edge
    AbsoluteCoefficient float64 // If non-singular, absolute coefficient of the line defining edge
}


// Create a new edge from vertices
// vertexStart is the starting point (vertex) of the polygon
// vertexEnd is the ending point (vertex) of the polygon
func NewEdge(vertexStart, vertexEnd *Vertex) (*Edge, error) {
    edge := &Edge{
        CoordXStart:         vertexStart.CoordX, // Horizontal coordinates of starting vertex
        CoordYStart:         vertexStart.CoordY, // Vertical coordinates of starting vertex
        CoordXEnd:           vertexEnd.CoordX,   // Horizontal coordinates of ending vertex
        CoordYEnd:           vertexEnd.CoordY,   // Vertical coordinates of ending vertex
        Singular:            true,               // If true, edge is either horizontal or vertical line
        SingularByX:         true,               // If singular and true, edge is horizontal line, vertical otherwise
        LinearCoefficient:   0.0,                // If non-singular, linear coefficient of the line defining edge
        AbsoluteCoefficient: 0.0,                // If non-singular, absolute coefficient of the line defining edge
    }

    // In the case of horizontal line
    if DeltaCompare(edge.CoordXStart, edge.CoordXEnd) {
        edge.SingularByX = false // If not horizontal then vertical line (or none if non-singular)
        if DeltaCompare(edge.CoordYStart, edge.CoordYEnd) {  // Edge cannot be a point!
            // Construct the error string
            var error_string strings.Builder
            error_string.WriteString("edge cannot be a point CoordX: (")
            error_string.WriteString(fmt.Sprintf("%f", edge.CoordXStart))
            error_string.WriteString(", ")
            error_string.WriteString(fmt.Sprintf("%f", edge.CoordXEnd))
            error_string.WriteString("), CoordY: (")
            error_string.WriteString(fmt.Sprintf("%f", edge.CoordYStart))
            error_string.WriteString(", ")
            error_string.WriteString(fmt.Sprintf("%f", edge.CoordYEnd))
            error_string.WriteString(")")
            return nil, errors.New(error_string.String())
        }
    }

    // Standard case (edge is described by the linear equation)
    if !DeltaCompare(edge.CoordXStart, edge.CoordXEnd) &&
        !DeltaCompare(edge.CoordYStart, edge.CoordYEnd) {
        edge.Singular = false // Not a horizontal or vertical line
        // lin_coef = (y_max - y_min) / (x_max - x_min)
        edge.LinearCoefficient = (edge.CoordYEnd - edge.CoordYStart) / (edge.CoordXEnd - edge.CoordXStart)
        // abs_coef = y_max - a * x_max
        edge.AbsoluteCoefficient = edge.CoordYEnd - edge.LinearCoefficient * edge.CoordXEnd
    }
    return edge, nil
}


// Compute with the edge of the ray coming from the point on position (CoordX, CoordY)
//  and defined by the linear and absolute coefficient (line).
// linearCoefficient is the linear coefficient of the ray coming from the point (x, y)
// absoluteCoefficient is the absolute coefficient of the ray coming from the point (x, y)
// x is the horizontal coordinates of the point from that ray is emitted
// y is the vertical coordinates of the point from that ray is emitted
// returns true if the ray intersect the edge
func (edge *Edge) RayIntersect(linearCoefficient, absoluteCoefficient, x, y float64) bool {
    if edge.Singular {  // The case when the edge is horizontal/vertical line
        if edge.SingularByX {  // Horizontal line:
            // Singular by CoordX (CoordX is point)
            x_val := (edge.CoordYEnd - absoluteCoefficient) / linearCoefficient
            // Remove corner cases (only one edge can be considered)
            if x_val == edge.CoordXStart {
                return false
            }
            // Analyse the intersection
            if x_val <= math.Max(edge.CoordXStart, edge.CoordXEnd) &&
                x_val >= math.Min(edge.CoordXStart, edge.CoordXEnd) {
                if x < x_val && y < edge.CoordYStart {
                    return true
                }
            }
        } else {  // Vertical line:
            // Singular by CoordY (CoordY point)
            // follows logic: CoordX = (CoordY - abs_coef) / lin_coef
            y_val := linearCoefficient* edge.CoordXEnd + absoluteCoefficient
            // Remove corner cases (only one edge can be considered)
            if y_val == edge.CoordYStart {
                return false
            }
            // Analyse the intersection
            if y_val <= math.Max(edge.CoordYStart, edge.CoordYEnd) &&
                y_val >= math.Min(edge.CoordYStart, edge.CoordYEnd) {
                if x < edge.CoordXStart && y < y_val {
                    return true
                }
            }
        }
        return false
    }

    // Singular case where lines are parallel
    if DeltaCompare(edge.LinearCoefficient, linearCoefficient) {
        if DeltaCompare(edge.AbsoluteCoefficient, absoluteCoefficient) {
            // Remove corner cases (only one edge can be considered)
            if x == edge.CoordXStart && y == edge.CoordYStart {
                return false
            }
            // Analyse the intersection
            if x >= math.Min(edge.CoordXStart, edge.CoordXEnd) &&
                x <= math.Max(edge.CoordXStart, edge.CoordXEnd) &&
                y <= math.Max(edge.CoordYStart, edge.CoordYEnd) &&
                y >= math.Min(edge.CoordYStart, edge.CoordYEnd) {
                return true
            }
        }
    }

    // else (typical case) - two non parallel lines (ray and line)
    x_val := (absoluteCoefficient - edge.AbsoluteCoefficient) / (edge.LinearCoefficient - linearCoefficient)
    y_val := edge.LinearCoefficient * x_val + edge.AbsoluteCoefficient

    // Remove corner cases
    if x_val == edge.CoordXStart && y_val == edge.CoordYStart {
        return false
    }

    // Computes if the intersection with lines lies on the edge
    if x_val <= math.Max(edge.CoordXStart, edge.CoordXEnd) && x_val >= math.Min(edge.CoordXStart, edge.CoordXEnd) {
        if y_val <= math.Max(edge.CoordYStart, edge.CoordYEnd) && y_val >= math.Min(edge.CoordYStart, edge.CoordYEnd) {
            if x < x_val && y < y_val {
                return true
            }
        }
    }
    return false
}


// Determine if the point (x, y) lies on the line
func (edge *Edge) OnLine(x, y float64) bool  {
    if edge.Singular {
        if edge.SingularByX {
            // Singular by CoordX (CoordY is point)
            if DeltaCompare(y, edge.CoordYEnd) {
                if x <= math.Max(edge.CoordXStart, edge.CoordXEnd) && x >= math.Min(edge.CoordXStart, edge.CoordXEnd) {
                    return true
                }
            }
        } else {
            // Singular by CoordY (CoordX point)
            if DeltaCompare(x, edge.CoordXEnd) {
                if y <= math.Max(edge.CoordYStart, edge.CoordYEnd) && y >= math.Min(edge.CoordYStart, edge.CoordYEnd) {
                    return true
                }
            }
        }
        return false
    }
    // else (typical case, non strictly horizontal/vertical line defined by the linear equation)
    y_val := edge.LinearCoefficient* x + edge.AbsoluteCoefficient
    if DeltaCompare(y, y_val) {
        if x <= math.Max(edge.CoordXStart, edge.CoordXEnd) && x >= math.Min(edge.CoordXStart, edge.CoordXEnd) {
            return true
        }
        if y <= math.Max(edge.CoordYStart, edge.CoordYEnd) && y >= math.Min(edge.CoordYStart, edge.CoordYEnd) {
            return true
        }
    }
    return false
}


// Polygon definition
type Polygon struct {
    BoundaryBox [4]float64  // Boundaries of the polygon (minimal rectangle containing polygon)
    Edges       []Edge      // Edges of the polygon (clock-wise logic)
}


// Create new polygon from vertices (sorted clockwise)
func NewPolygon(vertices []Vertex) (*Polygon, error) {
    if len(vertices) < 3 {
        return nil, errors.New("only a regular polygon is acceptable (with 3 or more vertices)")
    }
    // Variables for defining of the bounding box
    var x_max float64
    var x_min float64
    var y_max float64
    var y_min float64

    polygon := &Polygon{BoundaryBox: [4]float64{0, 0, 0, 0}, Edges: make([]Edge, len(vertices))}

    for i, vertex := range vertices {
        if i == 0 {
            // Assign first values to the border box
            x_max = vertex.CoordX
            x_min = vertex.CoordX
            y_max = vertex.CoordY
            y_min = vertex.CoordY
        } else {
            if new_edge, err := NewEdge(&vertices[i - 1], &vertex); err != nil {
                return nil, err
            } else {
                // If there is no error assign new edge
                polygon.Edges[i - 1] = *new_edge
            }
            // Re-compute bounding box
            if x_max < vertex.CoordX {
                x_max = vertex.CoordX
            }
            if y_max < vertex.CoordY {
                y_max = vertex.CoordY
            }
            if x_min > vertex.CoordX {
                x_min = vertex.CoordX
            }
            if y_min > vertex.CoordY {
                y_min = vertex.CoordY
            }
        }
    }
    polygon.BoundaryBox = [4]float64{x_max, x_min, y_max, y_min}

    if new_edge, err := NewEdge(&vertices[len(vertices) - 1], &vertices[0]); err != nil {
        return nil, err
    } else {
        // If there is no error assign new edge
        polygon.Edges[len(vertices) - 1] = *new_edge
    }

    return polygon, nil
}


// Implementation of the ray casting algorithm for solving of the Point In the Polygon (PIP) problem.
func (polygon *Polygon) PipRayCastingAlgorithm(x, y float64) bool {
    // Check the boundary box (if it is outside of BB, return false)
    if x > polygon.BoundaryBox[0] + 0.0001 {
        return false
    }
    if x < polygon.BoundaryBox[1] - 0.0001 {
        return false
    }
    if y > polygon.BoundaryBox[2] + 0.0001 {
        return false
    }
    if y < polygon.BoundaryBox[3] - 0.0001 {
        return false
    }
    // Probe the standard case
    linear_coefficient := 0.957218876
    absolute_coefficient := y - x * linear_coefficient
    counter_of_intersections := 0
    for _, edge := range polygon.Edges {
        // If the point is on the edge, return true
        if edge.OnLine(x, y) {
            return true
        }
        // Standard ray casting algorithm iteration
        if edge.RayIntersect(linear_coefficient, absolute_coefficient, x, y) {
            counter_of_intersections++
        }
    }
    // Ray Casting Algorithm -> if number of intersection mod 2 == 1, point is in the polygon
    if counter_of_intersections % 2 == 1 {
        return true
    }
    return false
}
