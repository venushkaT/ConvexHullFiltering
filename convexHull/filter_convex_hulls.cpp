#include "filter_convex_hulls.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

using namespace utils;
using namespace types;

void utils::SortVertices(std::vector<Point>& vertices) {
  double x_mean = 0;
  double y_mean = 0;
  int n_vertices = vertices.size();

  for (const auto vertice : vertices) {
    x_mean = x_mean + vertice.x / n_vertices;
    y_mean = y_mean + vertice.y / n_vertices;
  }

  std::sort(vertices.begin(), vertices.end(),
            [x_mean, y_mean](const auto vertice1, const auto vertice2) {
              double angle1 = atan2(vertice1.y - y_mean, vertice1.x - x_mean);
              double angle2 = atan2(vertice2.y - y_mean, vertice2.x - x_mean);
              return angle1 < angle2;
            });
}

bool utils::IsPointLeftSideOfLine(Point p, Line line) {
  p.x -= line.start.x;
  p.y -= line.start.y;

  line.end.x -= line.start.x;
  line.end.y -= line.start.y;

  return (line.end.x * p.y - line.end.y * p.x) > 0;
}

bool utils::IsPointInsideConvexHull(Point p, ConvexHull convex_hull) {
  std::vector<Line> lines = convex_hull.GetLines();

  // if p is on the left side of every line of convex hull,
  // then it is inside convex hull
  return std::all_of(lines.begin(), lines.end(), [p](const auto line) {
    return IsPointLeftSideOfLine(p, line);
  });
}

bool utils::LineSegmentIntersection(Line line1, Line line2,
                                    Point& intersection) {
  double Ax1 = line1.start.x;
  double Ay1 = line1.start.y;
  double Ax2 = line1.end.x;
  double Ay2 = line1.end.y;
  double Bx1 = line2.start.x;
  double By1 = line2.start.y;
  double Bx2 = line2.end.x;
  double By2 = line2.end.y;

  double d = (By2 - By1) * (Ax2 - Ax1) - (Bx2 - Bx1) * (Ay2 - Ay1);

  if (d == 0) {
    return false;
  } else {
    double uA = ((Bx2 - Bx1) * (Ay1 - By1) - (By2 - By1) * (Ax1 - Bx1)) / d;
    double uB = ((Ax2 - Ax1) * (Ay1 - By1) - (Ay2 - Ay1) * (Ax1 - Bx1)) / d;

    if (!(0 <= uA && uA <= 1 && 0 <= uB && uB <= 1)) {
      return false;
    }

    double x = Ax1 + uA * (Ax2 - Ax1);
    double y = Ay1 + uA * (Ay2 - Ay1);

    intersection = Point(x, y);

    return true;
  }
}

bool utils::ConvexHullIntersection(ConvexHull convex_hull1,
                                   ConvexHull convex_hull2,
                                   ConvexHull& intersection) {
  Point intersection_point;
  std::vector<Point> intersection_vertices;

  // for every permutation between lines in convex hull 1 and convex hull 2
  // the edges line1 and line2 chase each other, adjusting so that they meet at
  // each intersection
  for (const auto line1 : convex_hull1.GetLines()) {
    for (const auto line2 : convex_hull2.GetLines()) {
      // find the point of intersection, if exists
      if (LineSegmentIntersection(line1, line2, intersection_point)) {
        intersection_vertices.push_back(intersection_point);
      }
    }
  }

  // for every vertice in convex hull 1 check inside points
  for (const auto vertice : convex_hull1.GetVertices()) {
    // find if it is contained in convex hull 2
    if (IsPointInsideConvexHull(vertice, convex_hull2)) {
      intersection_vertices.push_back(vertice);
    }
  }

  // for every vertice in convex hull 2 check inside points
  for (const auto vertice : convex_hull2.GetVertices()) {
    // find if it is contained in convex hull 1
    if (IsPointInsideConvexHull(vertice, convex_hull1)) {
      intersection_vertices.push_back(vertice);
    }
  }

  // At the case of inadquate vertices count
  if (intersection_vertices.size() < 3) {
    return false;
  } else {
    intersection = ConvexHull(intersection_vertices);
    return true;
  }
}

Point::Point(double x, double y) {
  this->x = x;
  this->y = y;
}

Line::Line(Point start, Point end) {
  this->start = start;
  this->end = end;
}

void ConvexHull::SetLines(std::vector<Point> vertices) {
  if (!vertices.empty()) {
    lines_.push_back(Line(vertices.back(), vertices.at(0)));
    for (int idx = 0; idx < vertices.size() - 1; ++idx) {
      lines_.push_back(Line(vertices.at(idx), vertices.at(idx + 1)));
    }
  }
}

void ConvexHull::SetArea(std::vector<Point> vertices) {
  double left_sum = 0.0;
  double right_sum = 0.0;

  for (int i = 0; i < vertices.size(); ++i) {
    int j = (i + 1) % vertices.size();
    // compute with next vertices ,
    left_sum += vertices[i].x * vertices[j].y;
    right_sum += vertices[j].x * vertices[i].y;
  }

  area_ = 0.5 * abs(left_sum - right_sum);
}

ConvexHull::ConvexHull(std::vector<Point> vertices, int id) {
  if (vertices.size() >= 3) {
    id_ = id;
    vertices_ = vertices;
    // order vertices
    utils::SortVertices(vertices_);
    SetLines(vertices_);
    SetArea(vertices_);
  }
}
