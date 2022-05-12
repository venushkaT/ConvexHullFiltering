// This code follows Google C++ Style Guide.

#ifndef FILTER_CONVEX_HULLS_H_
#define FILTER_CONVEX_HULLS_H_

#include <vector>

namespace types {

struct Point {
  double x;
  double y;
  Point(){};
  Point(double x, double y);
};

struct Line {
  Point start;
  Point end;
  Line(){};
  Line(Point start, Point end);
};

class ConvexHull {
 private:
  int id_ = 0;
  std::vector<Point> vertices_;
  std::vector<Line> lines_;
  double area_ = 0;

  // find convexhull lines from pair of vertices
  void SetLines(std::vector<Point> vertices);

  // calculate area of convex hull from vertices using Shoelace formula
  //  [1] https://en.wikipedia.org/wiki/Shoelace_formula
  //  [2] https://rosettacode.org/wiki/Shoelace_formula_for_polygonal_area
  void SetArea(std::vector<Point> vertices);

 public:
  ConvexHull(){};
  ConvexHull(std::vector<Point> vertices, int id = -1);

  inline int GetID() { return id_; };
  inline std::vector<Point> GetVertices() { return vertices_; };
  inline std::vector<Line> GetLines() { return lines_; };
  inline double GetArea() { return area_; };
};
}  // namespace types

namespace utils {

// sort vertices counter clockwise
// [1] https://math.stackexchange.com/a/978648/421595
void SortVertices(std::vector<types::Point>& vertices);

// check if p is to the left side of line
// returns true if p is to the left side of line
// [1]
// https://algorithmtutor.com/Computational-Geometry/Check-if-a-point-is-inside-a-polygon/
bool IsPointLeftSideOfLine(types::Point p, types::Line line);

// check if p is inside convex hull
// returns true if p is inside convex hull
// [1]
// https://algorithmtutor.com/Computational-Geometry/Check-if-a-point-is-inside-a-polygon/
bool IsPointInsideConvexHull(types::Point p, types::ConvexHull convex_hull);

// find the intersection point between two line segments
// return true if point of intersection exists
// [1] https://rosettacode.org/wiki/Find_the_intersection_of_two_lines
bool LineSegmentIntersection(types::Line line1, types::Line line2,
                             types::Point& intersection);

// calculate the intersection hull formed by two convex hulls
// return true if intersection hull exists
// [1]
// https://tildesites.bowdoin.edu/~ltoma/teaching/cs3250-CompGeom/spring17/Lectures/cg-convexintersection.pdf
bool ConvexHullIntersection(types::ConvexHull convex_hull1,
                            types::ConvexHull convex_hull2,
                            types::ConvexHull& intersection);
}  // namespace utils

#endif FILTER_CONVEX_HULLS_H_