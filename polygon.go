package polygon_pip

import (
    "errors"
    "fmt"
    "math"
    "strings"
)


// Do the IEEE double precision float (64 bit) delta comparison
func DeltaCompare(a, b float64) bool {
    // Delta used for comparisons
    DELTA := 0.0001
    // Do comparison and return results
    return math.Abs(a - b) < DELTA
}


// Vertex (point) of the polygon.
type Vertex struct {
    CoordX float64 // Horizontal position of the point.
    CoordY float64 // Vertical position of the point.
}


// Edge (line) of the polygon. All edges has to be added clockwise.
type Edge struct {
    CoordXStart         float64 // horizontal (CoordX) axes of the starting vertex.
    CoordYStart         float64 // vertical (CoordY) axes of the starting vertex.
    CoordXEnd           float64 // horizontal (CoordX) axes of the ending vertex.
    CoordYEnd           float64 // vertical (CoordY) axes of the ending vertex.
    Singular            bool    // singular edge is horizontal or vertical.
    SingularByX         bool    // if true, edge is a vertical line, if false, horizontal, relevant only to Singular.
    LinearCoefficient   float64 // linear coefficient of the edge line.
    AbsoluteCoefficient float64 // absolute coefficient of the edge line.
}


// Create a new edge from vertices
func NewEdge(vertex_start, vertex_end *Vertex) (*Edge, error) {
    edge := &Edge{
        CoordXStart:         vertex_start.CoordX,
        CoordYStart:         vertex_start.CoordY,
        CoordXEnd:           vertex_end.CoordX,
        CoordYEnd:           vertex_end.CoordY,
        Singular:            true,
        SingularByX:         true, // Horizontal line
        LinearCoefficient:   0.0,
        AbsoluteCoefficient: 0.0}

    if DeltaCompare(edge.CoordXStart, edge.CoordXEnd) {
        edge.SingularByX = false // Vertical line
        if DeltaCompare(edge.CoordYStart, edge.CoordYEnd) {
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
func (edge *Edge) RayIntersect(linear_coefficient, absolute_coefficient, x, y float64) bool {
    if edge.Singular {
        if edge.SingularByX { // Horizontal line
            // Singular by CoordX (CoordX is point)
            x_val := (edge.CoordYEnd - absolute_coefficient) / linear_coefficient
            // Remove corner cases
            if x_val == edge.CoordXStart {
                return false
            }
            // On line case
            if x_val <= math.Max(edge.CoordXStart, edge.CoordXEnd) &&
                x_val >= math.Min(edge.CoordXStart, edge.CoordXEnd) {
                if x < x_val && y < edge.CoordYStart {
                    return true
                }
            }
        } else {  // Vertical line
            // Singular by CoordY (CoordY point)
            // follows logic: CoordX = (CoordY - abs_coef) / lin_coef
            y_val := linear_coefficient * edge.CoordXEnd + absolute_coefficient
            // Remove corner cases
            if y_val == edge.CoordYStart {
                return false
            }
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
    if DeltaCompare(edge.LinearCoefficient, linear_coefficient) {
        if DeltaCompare(edge.AbsoluteCoefficient, absolute_coefficient) {
            if x == edge.CoordXStart && y == edge.CoordYStart { // Remove corner cases
                return false
            }
            if x >= math.Min(edge.CoordXStart, edge.CoordXEnd) &&
                x <= math.Max(edge.CoordXStart, edge.CoordXEnd) &&
                y <= math.Max(edge.CoordYStart, edge.CoordYEnd) &&
                y >= math.Min(edge.CoordYStart, edge.CoordYEnd) {
                return true
            }
        }
    }

    // else (typical case) - two non parallel lines (ray and line)
    x_val := (absolute_coefficient - edge.AbsoluteCoefficient) / (edge.LinearCoefficient - linear_coefficient)
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
    // else (typical case)
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


type Polygon struct {
    boundary_box [4]float64
    edges []Edge
}


func NewPolygon(vertices []Vertex) (*Polygon, error) {
    if len(vertices) < 3 {
        return nil, errors.New("only a regular polygon is acceptable (with 3 or more vertices)")
    }
    // Variables for defining of the bounding box
    var x_max float64
    var x_min float64
    var y_max float64
    var y_min float64

    polygon := &Polygon{boundary_box: [4]float64{0, 0, 0, 0}, edges: make([]Edge, len(vertices))}

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
                polygon.edges[i - 1] = *new_edge
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
    polygon.boundary_box = [4]float64{x_max, x_min, y_max, y_min}

    if new_edge, err := NewEdge(&vertices[len(vertices) - 1], &vertices[0]); err != nil {
        return nil, err
    } else {
        // If there is no error assign new edge
        polygon.edges[len(vertices) - 1] = *new_edge
    }

    return polygon, nil
}

func (polygon *Polygon) PipRayCastingAlgorithm(x, y float64) bool {

    // Check the boundary box (if it is outside of BB, return false)
    if x > polygon.boundary_box[0] + 0.0001 {
        return false
    }
    if x < polygon.boundary_box[1] - 0.0001 {
        return false
    }
    if y > polygon.boundary_box[2] + 0.0001 {
        return false
    }
    if y < polygon.boundary_box[3] - 0.0001 {
        return false
    }
    // Probe the standard case
    linear_coefficient := 0.957218876
    absolute_coefficient := y - x * linear_coefficient
    counter_of_intersections := 0
    for _, edge := range polygon.edges {
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