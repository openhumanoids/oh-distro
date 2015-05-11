#include <vector>
#include <Eigen/Dense>

typedef double coord_t;         // coordinate type
typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2
 
struct Point {
  coord_t x, y;
 
  bool operator <(const Point &p) const {
    return x < p.x || (x == p.x && y < p.y);
  }
};

std::vector<Point> convex_hull(std::vector<Point> P);
bool in_convex_hull(std::vector<Point> P, Point q);
bool in_convex_hull(Eigen::Matrix<double, 2, Eigen::Dynamic> P, Eigen::Vector2d q);