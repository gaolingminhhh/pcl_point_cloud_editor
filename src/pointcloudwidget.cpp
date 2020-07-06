#include <pcl/apps/point_cloud_editor/pointcloudwidget.h>


PointCloudWidget::PointCloudWidget(QWidget *parent) : QVTKWidget(parent)
{
    m_visualizer=new pcl::visualization::PCLVisualizer("test");
    this->SetRenderWindow(m_visualizer->getRenderWindow());
}

PointCloudWidget::~PointCloudWidget()
{
    delete m_visualizer;
}
