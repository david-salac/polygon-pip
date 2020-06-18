from typing import List, Tuple


# Delta comparison value for determining strictly horizontal/vertical line.
DELTA = 0.0001


class Vertex(object):
    """Vertex (point) of the polygon.

    Attributes:
        x (float): Horizontal position of the point.
        y (float): Vertical position of the point.
    """
    def __init__(self, x: float, y: float):
        self.x: float = x
        self.y: float = y


class Edge(object):
    """Edge (line) of the polygon. All edges has to be added clockwise.

    Attributes:
        x_start (float): horizontal (x) axes of the starting vertex
        y_start (float): vertical (y) axes of the starting vertex
        x_end (float): horizontal (x) axes of the ending vertex
        y_end (float): vertical (y) axes of the ending vertex
        singular (bool): singular edge is horizontal or vertical
        singular_by_x (bool): if true, edge is a vertical line, if false,
            horizontal, relevant only to singular
        linear_coefficient (float): linear coefficient of the edge line
        absolute_coefficient (float): absolute coefficient of the edge line
    """

    def __init__(self,
                 x_start: float,
                 x_end: float,
                 y_start: float,
                 y_end: float):
        self.x_start: float = x_start
        self.y_start: float = y_start
        self.x_end: float = x_end
        self.y_end: float = y_end
        self.singular: bool = True
        self.singular_by_x: bool = True  # Horizontal line line
        self.linear_coefficient: float = None
        self.absolute_coefficient: float = None
        if abs(x_start - x_end) <= DELTA:
            self.singular_by_x = False  # Vertical line
            if abs(y_start - y_end) <= DELTA:
                raise ValueError("edge cannot be a point")

        if abs(x_start - x_end) > DELTA and abs(y_start - y_end) > DELTA:
            self.singular: bool = False  # Not a horizontal or vertical line
            # lin_coef = (y_max - y_min) / (x_max - x_min)
            self.linear_coefficient = (self.y_end - self.y_start) / \
                                      (self.x_end - self.x_start)
            # abs_coef = y_max - a * x_max
            self.absolute_coefficient = (self.y_end -
                                         self.linear_coefficient * x_end)

    @staticmethod
    def initialise_from_two_vertices(vertex_start: Vertex,
                                     vertex_end: Vertex) -> 'Edge':
        """Create instance of the edge from the two vertices comming clockwise.

        Args:
            vertex_start (Vertex): Starting vertex when going clockwise.
            vertex_end (Vertex): Ending vertex when going clockwise.
        """
        return Edge(vertex_start.x, vertex_end.x, vertex_start.y, vertex_end.y)

    def intersect(self,
                  linear_coefficient: float,
                  absolute_coefficient: float,
                  x: float,
                  y: float) -> bool:
        """Compute with the edge of the ray coming from the point on position
            (x, y) and defined by the linear and absolute coefficient (line).

        This method is important for ray casting algorithm (PIP algorithm,
            point in the polygon).

        Args:
            linear_coefficient (float): Linear coefficient of the ray
                (half-line)
            absolute_coefficient (float): Absolute coefficient of the ray
                (half-line)
            x (float): Horizontal position of the point.
            y (float): Vertical position of the point.

        Returns:
            bool: True if there is an intersection with the edge (excluding
                starting position of the edge).
        """
        if self.singular:
            if self.singular_by_x:  # Horizontal line
                # Singular by x (x is point)
                x_val = (self.y_end -
                         absolute_coefficient) / linear_coefficient
                # Remove corner cases
                if x_val == self.x_start:
                    return False
                # On line case
                if x_val <= max(self.x_start, self.x_end) \
                        and x_val >= min(self.x_start, self.x_end):
                    if x < x_val and y < self.y_start:
                        return True
                return False
            else:  # Vertical line
                # Singular by y (y point)
                # follows logic: x = (y - abs_coef) / lin_coef
                y_val = linear_coefficient * self.x_end + absolute_coefficient
                # Remove corner cases
                if y_val == self.y_start:
                    return False
                if y_val <= max(self.y_start, self.y_end) and \
                        y_val >= min(self.y_start, self.y_end):
                    if x < self.x_start and y < y_val:
                        return True
                return False
        # Singular case where lines are parallel
        if abs(self.linear_coefficient - linear_coefficient) < DELTA:
            if abs(self.absolute_coefficient - absolute_coefficient) < DELTA:
                # Remove corner cases
                if x == self.x_start and y == self.y_start:
                    return False
                if x >= min(self.x_start, self.x_end) and \
                        x <= max(self.x_start, self.x_end) and \
                        y >= min(self.y_start, self.y_end) and \
                        y <= max(self.y_start, self.y_end):
                    return True

        # Else (regular/typical cases)
        x_val = (absolute_coefficient - self.absolute_coefficient) / \
                (self.linear_coefficient - linear_coefficient)
        y_val = self.linear_coefficient * x_val + self.absolute_coefficient

        # Remove corner cases
        if x_val == self.x_start and y_val == self.y_start:
            return False

        if x_val <= max(self.x_start, self.x_end) and \
                x_val >= min(self.x_start, self.x_end):
            if y_val <= max(self.y_start, self.y_end) and \
                    y_val >= min(self.y_start, self.y_end):
                if x < x_val and y < y_val:
                    return True
        return False

    def on_line(self, x: float, y: float) -> bool:
        """Analyse if the point is on the edge.

        Args:
            x (float): Horizontal position of the point.
            y (float): Vertical position of the point.

        Returns:
            bool: True if there is the point lies on the edge.
        """
        if self.singular:
            if self.singular_by_x:
                # Singular by x (x is point)
                if abs(y - self.y_end) < DELTA:
                    if x <= max(self.x_start, self.x_end) and \
                            x >= min(self.x_start, self.x_end):
                        return True
            else:
                # Singular by y (y point)
                if abs(x - self.x_end) < DELTA:
                    if y <= max(self.y_start, self.y_end) and \
                            y >= min(self.y_start, self.y_end):
                        return True
            return False

        # Else (regular/typical cases)
        y_val = self.linear_coefficient * x + self.absolute_coefficient
        if abs(y - y_val) < DELTA:
            if x <= max(self.x_start, self.x_end) and \
                    x >= min(self.x_start, self.x_end):
                return True
            if y <= max(self.y_start, self.y_end) and \
                    y >= min(self.y_start, self.y_end):
                return True
        return False


class Polygon(object):
    """Polygon definition (as set of vertices and edges). Always defined
        in clockwise logic.

    Attributes:
        boundary_box (List[int, int, int, int]): The boundary box of the
            polygon (minimal rectangle such that the polygon is the subset
            of the rectangle). Indices are: x_max, x_min, y_max, y_min.
        edges (List[Edge]): A list of edges defined clockwise.

    """
    def __init__(self, vertices: Tuple[Vertex]):
        """Create polygon

        Args:
            vertices (Tuple[Vertex]): Clockwise defined set of vertices.
        """
        if len(vertices) < 3:
            raise ValueError("only regular polygons are accepted")

        x_max = 0
        x_min = 0
        y_max = 0
        y_min = 0
        self.boundary_box: List[int, int, int, int] = []

        previous_vertex: Vertex = None
        self.edges: List[Edge] = []
        for vertex in vertices:
            if previous_vertex is not None:
                self.edges.append(
                    Edge.initialise_from_two_vertices(previous_vertex, vertex)
                )
            previous_vertex = vertex
            if len(self.boundary_box) == 0:
                x_max = vertex.x
                y_max = vertex.y
                x_min = vertex.x
                y_min = vertex.y
                self.boundary_box = [x_max, x_min, y_max, y_min]
            else:
                if x_max < vertex.x:
                    x_max = vertex.x
                if y_max < vertex.y:
                    y_max = vertex.y
                if x_min > vertex.x:
                    x_min = vertex.x
                if y_min > vertex.y:
                    y_min = vertex.y
                self.boundary_box = [x_max, x_min, y_max, y_min]
        # Last edge:
        self.edges.append(
            Edge.initialise_from_two_vertices(previous_vertex, vertices[0])
        )

    def pip_ray_casting_algorithm(self, x: float, y: float) -> bool:
        """Implementation of the ray casting algorithm.

        Simple algorithm that determines if the point is inside of the polygon
            or not (in this case the edge cases is considered to be inside).

        Args:
            x (float): Horizontal position of the point.
            y (float): Vertical position of the point.

        Returns:
            bool: True if the point is inside of the polygon or on the border
                of the polygon. False if it is outside of the polygon.

        Warning:
            If the point is on the coordinates that are multiplication
                of the value 0.957218876 you can receive a wrong answer.
        """
        # Check the boundary box (if it is outside of BB, return false)
        if x > self.boundary_box[0]:
            return False
        if x < self.boundary_box[1]:
            return False
        if y > self.boundary_box[2]:
            return False
        if y < self.boundary_box[3]:
            return False
        # Probe the standard case
        linear_coefficient = 0.957218876
        absolute_coefficient = y - x * linear_coefficient
        counter_of_intersections = 0
        for edge in self.edges:
            if edge.on_line(x, y):
                # Special case when point is on the edge
                return True
            if edge.intersect(linear_coefficient, absolute_coefficient, x, y):
                counter_of_intersections += 1

        if counter_of_intersections % 2 == 1:
            return True
        return False
