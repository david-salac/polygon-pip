package polygon_pip

import (
    "errors"
    "fmt"
    "math"
    "strings"
)


// Do the IEEE double precision float (64 bit) delta comparison
func deltaCompare(a, b float64) bool {
    // Delta used for comparisons
    DELTA := 0.0001
    // Do comparison and return results
    return math.Abs(a - b) < DELTA
}


// Vertex (point) of the polygon.
type Vertex struct {
    x float64  // Horizontal position of the point.
    y float64  // Vertical position of the point.
}


// Edge (line) of the polygon. All edges has to be added clockwise.
type Edge struct {
    x_start float64  // horizontal (x) axes of the starting vertex.
    y_start float64  // vertical (y) axes of the starting vertex.
    x_end float64    // horizontal (x) axes of the ending vertex.
    y_end float64    // vertical (y) axes of the ending vertex.
    singular bool    // singular edge is horizontal or vertical.
    singular_by_x bool  // if true, edge is a vertical line, if false, horizontal, relevant only to singular.
    linear_coefficient float64    // linear coefficient of the edge line.
    absolute_coefficient float64  // absolute coefficient of the edge line.
}


// Create a new edge from vertices
func NewEdge(vertex_start, vertex_end *Vertex) (*Edge, error) {
    edge := &Edge{
        x_start: vertex_start.x,
        y_start: vertex_start.y,
        x_end: vertex_end.x,
        y_end: vertex_end.y,
        singular: true,
        singular_by_x: true, // Horizontal line
        linear_coefficient: 0.0,
        absolute_coefficient: 0.0}

    if deltaCompare(edge.x_start, edge.x_end) {
        edge.singular_by_x = false  // Vertical line
        if deltaCompare(edge.y_start, edge.y_end) {
            var error_string strings.Builder
            error_string.WriteString("edge cannot be a point x: (")
            error_string.WriteString(fmt.Sprintf("%f", edge.x_start))
            error_string.WriteString(", ")
            error_string.WriteString(fmt.Sprintf("%f", edge.x_end))
            error_string.WriteString("), y: (")
            error_string.WriteString(fmt.Sprintf("%f", edge.y_start))
            error_string.WriteString(", ")
            error_string.WriteString(fmt.Sprintf("%f", edge.y_end))
            error_string.WriteString(")")
            return nil, errors.New(error_string.String())
        }
    }
    if !deltaCompare(edge.x_start, edge.x_end) && !deltaCompare(edge.y_start, edge.y_end) {
        edge.singular = false  // Not a horizontal or vertical line
        // lin_coef = (y_max - y_min) / (x_max - x_min)
        edge.linear_coefficient = (edge.y_end - edge.y_start) / (edge.x_end - edge.y_start)
        // abs_coef = y_max - a * x_max
        edge.absolute_coefficient = edge.y_end - edge.linear_coefficient * edge.x_end
    }
    return edge, nil
}


// Compute with the edge of the ray coming from the point on position (x, y)
//  and defined by the linear and absolute coefficient (line).
func (edge *Edge) RayIntersect(linear_coefficient, absolute_coefficient, x, y float64) bool {
    if edge.singular {
        if edge.singular_by_x {  // Horizontal line
            // Singular by x (x is point)
            x_val := (edge.y_end - absolute_coefficient) / linear_coefficient
            // Remove corner cases
            if x_val == edge.x_start {
                return false
            }
            // On line case
            if x_val <= math.Max(edge.x_start, edge.x_end) && x_val >= math.Min(edge.x_start, edge.x_end) {
                if x < x_val && y < edge.y_start {
                    return true
                }
            }
        } else {  // Vertical line
            // Singular by y (y point)
            // follows logic: x = (y - abs_coef) / lin_coef
            y_val := linear_coefficient * edge.x_end + absolute_coefficient
            // Remove corner cases
            if y_val == edge.y_start {
                return false
            }
            if y_val <= math.Max(edge.y_start, edge.y_end) && y_val >= math.Min(edge.y_start, edge.y_end) {
                if x < edge.x_start && y < y_val {
                    return true
                }
            }
        }
        return false
    }

    // Singular case where lines are parallel
    if deltaCompare(edge.linear_coefficient, linear_coefficient) {
        if deltaCompare(edge.absolute_coefficient, absolute_coefficient) {
            if x == edge.x_start && y == edge.y_start {  // Remove corner cases
                return false
            }
            if x >= math.Min(edge.x_start, edge.x_end) && y >= math.Min(edge.y_start, edge.y_end) {
                return true
            }
        }
    }

    // else (typical case) - two non parallel lines (ray and line)
    x_val := (absolute_coefficient - edge.absolute_coefficient) / (edge.linear_coefficient - linear_coefficient)
    y_val := edge.linear_coefficient * x_val + edge.absolute_coefficient

    // Remove corner cases
    if x_val == edge.x_start && y_val == edge.y_start {
        return false
    }

    // Computes if the intersection with lines lies on the edge
    if x_val <= math.Max(edge.x_start, edge.x_end) && x_val >= math.Min(edge.x_start, edge.x_end) {
        if y_val <= math.Max(edge.y_start, edge.y_end) && y_val >= math.Min(edge.y_start, edge.y_end) {
            if x < x_val && y < y_val {
                return true
            }
        }
    }
    return false
}

func (edge *Edge) OnLine(x, y float64) bool  {
    if edge.singular {
        if edge.singular_by_x {
            // Singular by x (x is point)
            if deltaCompare(y, edge.y_end) {
                if x <= math.Max(edge.x_start, edge.x_end) && x >= math.Min(edge.x_start, edge.x_end) {
                    return true
                }
            }
        } else {
            // Singular by y (y point)
            if deltaCompare(x, edge.x_end) {
                if y <= math.Max(edge.y_start, edge.y_end) && y >= math.Min(edge.y_start, edge.y_end) {
                    return true
                }
            }
        }
        return false
    }
    // else (typical case)
    y_val := edge.linear_coefficient * x + edge.absolute_coefficient
    if deltaCompare(y, y_val) {
        if x <= math.Max(edge.x_start, edge.x_end) && x >= math.Min(edge.x_start, edge.x_end) {
            return true
        }
        if y <= math.Max(edge.y_start, edge.y_end) && y >= math.Min(edge.y_start, edge.y_end) {
            return true
        }
    }
    return false
}


type Polygon struct {
    boundary_box [4] float64
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
            x_max = vertex.x
            x_min = vertex.x
            y_max = vertex.y
            y_min = vertex.y
        } else {
            if new_edge, err := NewEdge(&vertices[i - 1], &vertex); err != nil {
                return nil, err
            } else {
                // If there is no error assign new edge
                polygon.edges[i - 1] = *new_edge
            }
            // Re-compute bounding box
            if x_max < vertex.x {
                x_max = vertex.x
            }
            if y_max < vertex.y {
                y_max = vertex.y
            }
            if x_min > vertex.x {
                x_min = vertex.x
            }
            if y_min > vertex.y {
                y_min = vertex.y
            }
        }
    }
    polygon.boundary_box = [4]float64{x_max, x_min, y_max, y_min}

    if new_edge, err := NewEdge(&vertices[len(vertices) - 2], &vertices[len(vertices) - 1]); err != nil {
        return nil, err
    } else {
        // If there is no error assign new edge
        polygon.edges[len(vertices) - 1] = *new_edge
    }

    return polygon, nil
}

func (polygon *Polygon) PipRayCastingAlgorithm(x, y float64) bool {

    // Check the boundary box (if it is outside of BB, return false)
    if x > polygon.boundary_box[0] {
        return false
    }
    if x < polygon.boundary_box[1] {
        return false
    }
    if y > polygon.boundary_box[2] {
        return false
    }
    if y < polygon.boundary_box[3] {
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