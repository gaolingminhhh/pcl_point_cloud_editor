#ifndef BESTFITPLANE_H
#define BESTFITPLANE_H
#include <Eigen/Dense>
#include <Eigen/Core>
#include <utility>
#include <vector>
using namespace Eigen;
class BestFitPlane
{
  public:
    BestFitPlane(){}
    std::pair<Vector3f, Vector3f> best_plane_from_points(const std::vector<Vector3f> & c);
};

#endif // BESTFITPLANE_H
