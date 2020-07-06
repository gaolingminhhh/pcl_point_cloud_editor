#include <pcl/apps/point_cloud_editor/bestfitplane.h>
#include <QDebug>

std::pair<Vector3f, Vector3f>
BestFitPlane::best_plane_from_points(const std::vector<Vector3f> &c)
{
    size_t num_atoms = c.size();
    Eigen::Matrix< Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; ++i)
        coord.col(i) = c[i];

    // calculate centroid
    Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid(0);
    coord.row(1).array() -= centroid(1);
    coord.row(2).array() -= centroid(2);

    // we only need the left-singular matrix here
    auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Vector3f plane_normal = svd.matrixU().rightCols<1>();
    return std::make_pair(centroid, plane_normal);
}
