#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>

#include "filter_convex_hulls.h"

using namespace types;
using namespace utils;
using json = nlohmann::json;
using namespace std;

// Get filtered list of convex hulls that do not
// overlap more than 50% with other convex hulls
void FilterConvexHulls(std::vector<ConvexHull> convex_hulls,
                       std::vector<ConvexHull>& filtered_convex_hulls) {
  // for each convex hull, get one as reference
  for (auto& convex_hull_ref : convex_hulls) {
    // total overlap ratio of reference convex hull
    double overlap_ratio = 0;

    // for each convex hull, Calculate intersection with reference convex hull
    for (auto& convex_hull : convex_hulls) {
      // Avoid calculating intersection of itself
      if (convex_hull_ref.GetID() != convex_hull.GetID()) {
        // calculate the intersection hull formed by two convex hulls
        ConvexHull intersection_hull;
        if (ConvexHullIntersection(convex_hull_ref, convex_hull,
                                   intersection_hull)) {
          // calculate overlap ratio between reference
          // convex hull and intersection hull
          double intersection_area = intersection_hull.GetArea();
          double convex_hull_area = convex_hull_ref.GetArea();
          overlap_ratio += intersection_area / convex_hull_area;
        }
      }
    }

    // if total overlap ratio is <= 50%, then push
    // convex hull info to filtered_convex_hulls
    if (overlap_ratio <= .5) {
      filtered_convex_hulls.push_back(convex_hull_ref);
    }
  }
}

int main() {
  // read input JSON file
  std::ifstream json_file_in("convex_hulls.json");
  json convex_hulls_json_in;
  json_file_in >> convex_hulls_json_in;

  // convert JSON object to vector of ConvexHulls
  std::vector<ConvexHull> convex_hulls;
  for (const auto convex_hull : convex_hulls_json_in["convex hulls"]) {
    std::vector<Point> vertices;
    for (const auto vertice : convex_hull["apexes"]) {
      vertices.push_back(Point(vertice["x"], vertice["y"]));
    }
    // create lines from vertices
    convex_hulls.push_back(ConvexHull(vertices, convex_hull["ID"]));
  }

  // get vector of filtered convex hulls
  std::vector<ConvexHull> filtered_convex_hulls;
  FilterConvexHulls(convex_hulls, filtered_convex_hulls);

  // convert vector of filter convex hulls to JSON object
  std::vector<json> filtered_convex_hulls_json;
  for (auto convex_hull : filtered_convex_hulls) {
    std::vector<json> apexes;
    for (const auto vertice : convex_hull.GetVertices()) {
      apexes.push_back(json({{"x", vertice.x}, {"y", vertice.y}}));
    }

    filtered_convex_hulls_json.push_back(
        json({{"ID", convex_hull.GetID()}, {"apexes", apexes}}));
  }

  // add filtered_convex_hulls_json to "result convex hulls" field
  json convex_hulls_json_out;  // output json
  convex_hulls_json_out["result convex hulls"] = filtered_convex_hulls_json;

  // write a results JSON file
  // [1] https://github.com/nlohmann/json#tofrom-streams-eg-files-string-streams
  std::ofstream json_file_out("result_convex_hulls.json");
  json_file_out << std::setw(3) << convex_hulls_json_out << std::endl;
}