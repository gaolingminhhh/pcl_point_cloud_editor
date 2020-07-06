#ifndef INTERACTIVE_PANEL_H
#define INTERACTIVE_PANEL_H
#include <QLineEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QTimer>
#include <QDebug>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <QCheckBox>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <QPushButton>
#include <pcl/registration/icp.h>
#include <pcl/apps/point_cloud_editor/highlightpoints.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>//包含fpfh加速计算的omp多核并行计算

#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/point_representation.h>


typedef pcl::FPFHSignature33 FPFHT;
typedef pcl::PointCloud<FPFHT> FPFHCloud;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

class Interactive_Panel:public QDialog
{
    Q_OBJECT
public:
    Interactive_Panel(
            CloudPtr cloud_in,
            CloudPtr cloud_out,
            boost::shared_ptr<HightLightPoints> highlight,
            QWidget *parent = nullptr);
    ~Interactive_Panel();

    const float VOXEL_GRID_SIZE = 0.01;
    const double radius_normal=30;
    const double radius_feature=30;
    const double max_sacia_iterations=1000;
    const double min_correspondence_dist=0.01;
    const double max_correspondence_dist=1000;

public Q_SLOTS:

private Q_SLOTS:
    void
    ChangeColor(int i);

    void
    InverseColor(int i);

    void
    accept () override;



private:
    void voxelFilter(PclCloudPtr &cloud_in, PclCloudPtr &cloud_out, float gridsize);
    pcl::PointCloud<pcl::Normal>::Ptr getNormals(PclCloudPtr cloud, double radius);
    FPFHCloud::Ptr getFeatures(PointCloud::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,double radius);
    void sac_ia_align(PclCloudPtr source,PclCloudPtr target,PclCloudPtr finalcloud,Eigen::Matrix4f init_transform,
                      int max_sacia_iterationss,double min_correspondence_dists,double max_correspondence_dists);
    void pairAlign (const PclCloudPtr cloud_src, const PclCloudPtr cloud_tgt, PclCloudPtr output, Eigen::Matrix4f &final_transform, bool downsample = false);
    void getMaxMin(PclCloudPtr cloud,float max_xyz_[XYZ_SIZE],float min_xyz_[XYZ_SIZE]);
    float getMinLength(float max_xyz[],float min_xyz[]);
  //  pcl::search::KdTree<PointT>::Ptr tree;// (new pcl::search::KdTree<PointT> ());
    CloudPtr cloud_src_ptr;
    CloudPtr cloud_tgt_ptr;

    QCheckBox *checkbox;
    QCheckBox *checkbox2;
    QDialogButtonBox *closeBtn;
    boost::shared_ptr<HightLightPoints> highlight;
    pcl::IterativeClosestPoint<Point3D, Point3D> icp;
    IndexVector hightlightindicies;
    IndexVector hightlightindicies2;
    float tgt_max_xyz[XYZ_SIZE];
    float tgt_min_xyz[XYZ_SIZE];
    float src_max_xyz[XYZ_SIZE];
    float src_min_xyz[XYZ_SIZE];
    unsigned int cloudsize=0;
    float length_tgt,length_src;
    float pieceCount=10;
};

#endif // INTERACTIVE_PANEL_H
