#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <iostream>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <QVTKWidget.h>
class PointCloudWidget : public QVTKWidget
{
    //Whatever comes before (constructor, methods, etc.)
public:
    PointCloudWidget(QWidget *parent);
    ~PointCloudWidget();
private:

    pcl::visualization::PCLVisualizer *m_visualizer;
};
#endif // POINTCLOUDWIDGET_H
