#include <Eigen/Dense>
#include <vector>

namespace viz {
struct Point {
  Eigen::Vector3d x;
  unsigned char r,g,b;
};

struct Pointcloud {
  std::vector<viz::Point> points;
};

struct BoundingBox{
  Eigen::Vector3d dimensions;
  Eigen::Vector3d translation;
  unsigned char r,g,b;
  double rotation;
};
}
