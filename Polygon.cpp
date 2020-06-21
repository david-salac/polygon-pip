#include "Polygon.hpp"


/**
 * Delta comparison of double
 * @param a first operand for comparison
 * @param b second operand for comparison
 * @return abs(a - b) < DELTA
 */
inline bool DeltaCompare(double a, double b, double delta = DELTA) {
    // Do comparison and return results
    return ((a - b) > 0 ? (a - b) : (-1) * (a - b)) < delta;
}


/**
 * Create new instance of Edge
 * @param start starting point (vertex)
 * @param end ending point (vertex)
 */
Edge::Edge(Vertex start, Vertex end) :
    coordXStart(start.coordX),
    coordYStart(start.coordY),
    coordXEnd(end.coordX),
    coordYEnd(end.coordY) {
    // Compute other fields:
    this->singular = true;
    this->singularByX = true;
    this->linearCoefficient = 0;
    this->absoluteCoefficient = 0;

    // In the case of horizontal line
    if (DeltaCompare(this->coordXStart, this->coordXEnd)) {
        this->singularByX = false; // If not horizontal then vertical line (or none if non-singular)
        if (DeltaCompare(this->coordYStart, this->coordYEnd)) {  // Edge cannot be a point!
            // TODO: Raise exception
        }
    }
    // Standard case (edge is described by the linear equation)
    if (!DeltaCompare(this->coordXStart, this->coordXEnd) && !DeltaCompare(this->coordYStart, this->coordYEnd)) {
        this->singular = false; // Not a horizontal or vertical line
        // lin_coef = (y_max - y_min) / (x_max - x_min)
        this->linearCoefficient = (this->coordYEnd - this->coordYStart) / (this->coordXEnd - this->coordXStart);
        // abs_coef = y_max - a * x_max
        this->absoluteCoefficient = this->coordYEnd - this->linearCoefficient * this->coordXEnd;
    }
}


/**
 * Create new edge from tuples of coordinates
 * @param start position (x, y) of start
 * @param end  position (x, y) of end
 */
Edge::Edge(std::tuple<double, double> start, std::tuple<double, double> end) :
    Edge(Vertex(std::get<0>(start), std::get<1>(start)), Vertex(std::get<0>(end), std::get<1>(end))) {
}


/**
 * Compute with the edge of the ray coming from the point on position (CoordX, CoordY)
 * and defined by the linear and absolute coefficient (line).
 * @param linearCoefficient the linear coefficient of the ray coming from the point (x, y)
 * @param absoluteCoefficient the absolute coefficient of the ray coming from the point (x, y)
 * @param x coordinates of the point from that ray is emitted
 * @param y coordinates of the point from that ray is emitted
 * @return true if the ray intersect the edge, false otherwise
 */
bool Edge::RayIntersect(double linearCoefficient, double absoluteCoefficient, double x, double y) const {
    if (this->singular) {  // The case when the edge is horizontal/vertical line
        if (this->singularByX) {  // Horizontal line:
            // Singular by CoordX (CoordX is point)
            double x_val = (this->coordYEnd - absoluteCoefficient) / linearCoefficient;
            // Remove corner cases (only one edge can be considered)
            if (x_val == this->coordXStart) {
                return false;
            }
            // Analyse the intersection
            if (x_val <= std::max(this->coordXStart, this->coordXEnd) &&
               x_val >= std::min(this->coordXStart, this->coordXEnd)) {
                if (x < x_val && y < this->coordYStart) {
                    return true;
                }
            }
        } else {  // Vertical line:
            // Singular by CoordY (CoordY point)
            // follows logic: CoordX = (CoordY - abs_coef) / lin_coef
            double y_val = linearCoefficient * this->coordXEnd + absoluteCoefficient;
            // Remove corner cases (only one edge can be considered)
            if (y_val == this->coordYStart) {
                return false;
            }
            // Analyse the intersection
            if (y_val <= std::max(this->coordYStart, this->coordYEnd) &&
               y_val >= std::min(this->coordYStart, this->coordYEnd)) {
                if (x < this->coordXStart && y < y_val) {
                            return true;
                    }
            }
        }
        return false;
    }

    // Singular case where lines are parallel
    if (DeltaCompare(this->linearCoefficient, linearCoefficient)) {
        if (DeltaCompare(this->absoluteCoefficient, absoluteCoefficient)) {
            // Remove corner cases (only one edge can be considered)
            if (x == this->coordXStart && y == this->coordYStart) {
                return false;
            }
            // Analyse the intersection
            if (x >= std::min(this->coordXStart, this->coordXEnd) &&
               x <= std::max(this->coordXStart, this->coordXEnd) &&
               y <= std::max(this->coordYStart, this->coordYEnd) &&
               y >= std::min(this->coordYStart, this->coordYEnd)) {
                return true;
            }
        }
    }

    // else (typical case) - two non parallel lines (ray and line)
    double x_val = (absoluteCoefficient - this->absoluteCoefficient) / (this->linearCoefficient - linearCoefficient);
    double y_val = this->linearCoefficient * x_val + this->absoluteCoefficient;

    // Remove corner cases
    if (x_val == this->coordXStart && y_val == this->coordYStart) {
        return false;
    }

    // Computes if the intersection with lines lies on the edge
    if (x_val <= std::max(this->coordXStart, this->coordXEnd) && x_val >= std::min(this->coordXStart, this->coordXEnd)) {
        if (y_val <= std::max(this->coordYStart, this->coordYEnd) && y_val >= std::min(this->coordYStart, this->coordYEnd)) {
            if (x < x_val && y < y_val) {
                return true;
            }
        }
    }
    return false;
}


/**
 * Determine if the point (x, y) lies on the line
 * @param x horizontal position of the point
 * @param y vertical position of the point
 * @return true if the edge is on the line, false otherwise
 */
bool Edge::OnLine(double x, double y) const {
    if (this->singular) {
        if (this->singularByX) {
            // Singular by CoordX (CoordY is point)
            if (DeltaCompare(y, this->coordYEnd)) {
                if (x <= std::max(this->coordXStart, this->coordXEnd) && x >= std::min(this->coordXStart, this->coordXEnd)) {
                    return true;
                }
            }
        } else {
            // Singular by CoordY (CoordX point)
            if (DeltaCompare(x, this->coordXEnd)) {
                if (y <= std::max(this->coordYStart, this->coordYEnd) && y >= std::min(this->coordYStart, this->coordYEnd)) {
                    return true;
                }
            }
        }
        return false;
    }
    // else (typical case, non strictly horizontal/vertical line defined by the linear equation)
    double y_val = this->linearCoefficient * x + this->absoluteCoefficient;
    if (DeltaCompare(y, y_val)) {
        if (x <= std::max(this->coordXStart, this->coordXEnd) && x >= std::min(this->coordXStart, this->coordXEnd)) {
            return true;
        }
        if (y <= std::max(this->coordYStart, this->coordYEnd) && y >= std::min(this->coordYStart, this->coordYEnd)) {
            return true;
        }
    }
    return false;
}


/**
 * Create new polygon
 * @param vertices Array of tuples with x, y coordinates.
 */
Polygon::Polygon(std::vector<std::tuple<double, double>> vertices):
    boundaryBox(std::array<double, 4>()),
    edges(std::vector<Edge>()) {

    if (vertices.size() < 3) {
        // TODO: Raise exception
    }
    // Variables for defining of the bounding box
    double x_max;
    double x_min;
    double y_max;
    double y_min;


    for (std::size_t i = 0; i < vertices.size(); ++i) {
        std::tuple<double, double> vertex = vertices[i];
        if (i == 0) {
            // Assign first values to the border box
            x_max = std::get<0>(vertex);
            x_min = std::get<0>(vertex);
            y_max = std::get<1>(vertex);
            y_min = std::get<1>(vertex);
        } else {
            // Assign new edge
            this->edges.push_back(Edge(vertices[i - 1], vertex));
            // Re-compute bounding box
            if (x_max < std::get<0>(vertex)) {
                x_max = std::get<0>(vertex);
            }
            if (y_max < std::get<1>(vertex)) {
                y_max = std::get<1>(vertex);
            }
            if (x_min > std::get<0>(vertex)) {
                x_min = std::get<0>(vertex);
            }
            if (y_min > std::get<1>(vertex)) {
                y_min = std::get<1>(vertex);
            }
        }
    }
    this->boundaryBox = {x_max, x_min, y_max, y_min};
    // Create the last edge (from the last point to the first one
    this->edges.push_back(Edge(vertices[vertices.size() - 1], vertices[0]));
}


/**
 * Point In the Polygon (PIP) algorithm (ray casting algorithm version).
 * @param x horizontal position
 * @param y vertical position
 * @return true if the point is inside of the polygon or on the edge, false otherwise
 */
bool Polygon::PipRayCastingAlgorithm(double x, double y) const {
    // Check the boundary box (if it is outside of BB, return false)
    if (x > this->boundaryBox[0] + DELTA) {
        return false;
    }
    if (x < this->boundaryBox[1] - DELTA) {
        return false;
    }
    if (y > this->boundaryBox[2] + DELTA) {
        return false;
    }
    if (y < this->boundaryBox[3] - DELTA) {
        return false;
    }
    // Probe the standard case
    double linear_coefficient = 0.957218876;
    double absolute_coefficient = y - x * linear_coefficient;
    int counter_of_intersections = 0;

    for (int i = 0; i < this->edges.size(); ++i) {
        // Point on the edge
        if (this->edges[i].OnLine(x, y)) {
            return true;
        }
        // Standard ray casting algorithm iteration
        if (this->edges[i].RayIntersect(linear_coefficient, absolute_coefficient, x, y)) {
            ++counter_of_intersections;
        }
    }
    // Ray Casting Algorithm -> if number of intersection mod 2 == 1, point is in the polygon
    if ((counter_of_intersections % 2) == 1) {
        return true;
    }
    return false;
}


/**
 * Return the reference to edges
 * @return edges in the polygon
 */
const std::vector<Edge>& Polygon::Edges() const {
    return this->edges;
}


/**
 * Get starting and ending vertex defining the edge
 * @return starting and ending vertex defining the edge
 */
std::tuple<Vertex, Vertex> Edge::Vertices() const {
    return std::make_tuple(
        Vertex{this->coordXStart, this->coordYStart},
        Vertex{this->coordXEnd, this->coordYEnd}
    );
}
