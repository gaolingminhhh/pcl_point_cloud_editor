#include <pcl/apps/point_cloud_editor/ranging.h>
#include <algorithm>
#include <math.h>
#include <QWidget>
#include <pcl/apps/point_cloud_editor/mainWindow.h>
#include <QToolTip>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/pcl_base.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

Ranging::Ranging(boost::shared_ptr<Converter> converter,CloudPtr cloud_ptr_,boost::shared_ptr<HightLightPoints> highLighter)
{
    this->converter=converter;
    this->cloud_ptr_=cloud_ptr_;
    this->highLighter=highLighter;
    calculateobb();
    xozNormal=calculateNormal(x_axis,z_axis,center);
    yozNormal=calculateNormal(y_axis,z_axis,center);
    xoyNormal=calculateNormal(x_axis,y_axis,center);
    qDebug("初始化完毕");
}

Ranging::~Ranging(){}

float
Ranging::countTriangleArea(Point a, Point b, Point c)
{
    float side[3];//存储三条边的长度;

    side[0] = sqrt(pow(a.x - b.x,2)+pow(a.y - b.y,2) + pow(a.z - b.z,2));
    side[1] = sqrt(pow(a.x - c.x,2)+pow(a.y - c.y,2) + pow(a.z - c.z,2));
    side[2] = sqrt(pow(c.x - b.x,2)+pow(c.y - b.y,2) + pow(c.z - b.z,2));
    float area=0;
    //不能构成三角形;
    if(side[0]+side[1]<=side[2] || side[0]+side[2]<=side[1] || side[1]+side[2]<=side[0])
    {
        area=0;
        return area;
    }

    //利用海伦公式。s=sqr(p*(p-a)(p-b)(p-c));
    double p = (side[0]+side[1]+side[2])/2; //半周长;
    area = sqrt(p*(p-side[0])*(p-side[1])*(p-side[2]));
    return area;
}

void
Ranging::getPoint3D(int x, int y,QPointF screen_pos,QWidget *widget)
{
    qDebug("getPoint3D");
    Point3D point;
    int index=0;
    if(converter->getDepthValue(x,y,point,index))
    {
        points.push_back(Point(point.x,point.y,point.z));
        screenPositions.push_back(screen_pos);
        qDebug("index : %d",index);
        indicies.push_back(index);
        highLight(index);
        times++;
    }
    qDebug("getPoint3D Finished");

}

void
Ranging::highLight(int index)
{
    highLighter->highlightsinglepoint(index);
}

void
Ranging::reset()
{
    qDebug("在ranging中重置");
    for (int i=0;i<times;i++) {
        highLighter->dishighlight(indicies[i]);
    }
    screenPositions.clear();
    points.clear();
    indicies.clear();
    times=0;
    xoyArea=0;
    xozArea=0;
    yozArea=0;
    perimeter=0;

}

float
Ranging::getDistance(Point point1, Point point2)
{
    float disx=std::abs(point1.x-point2.x);
    float disy=std::abs(point1.y-point2.y);
    float disz=std::abs(point1.z-point2.z);
    qDebug("%f %f %f %f %f %f",point1.x,point1.y,point1.z,point2.x,point2.y,point2.z);

    float distance=sqrt(disx*disx+disy*disy+disz*disz);
    qDebug("distance : %f",distance);
    return distance;
}

void
Ranging::onMouseReleased(int x,int y,const QPointF screen_pos,QWidget *widget)
{
    getPoint3D(x,y,screen_pos,widget);
    update();
    qDebug("鼠标释放事件");
    QString perimeterstr("周长:");
    QString areaStr("面积:");
    if(times>=3)
        ((MainWindow*)widget)->SetArea(areaStr.append(QString::number(xoyArea)));
    ((MainWindow*)widget)->SetPerimeter(perimeterstr.append(QString::number(perimeter)));
}

void
Ranging::undo()
{
    points.pop_back();
    screenPositions.pop_back();
    update();
}

void Ranging::onMousePressed(int x,int y)
{
}

void
Ranging::drawLine(QPointF point1, QPointF point2)
{
    //TODO 画出线
    qDebug("画线");
}

void
Ranging::calculateArea()
{
    qDebug("计算面积");
    qDebug("times=%d",times);
    xozArea=0;
    yozArea=0;
    xoyArea=0;
    //当非闭合图像时,返回
    if(times<=2)
        return;
    Point center_(center.x,center.y,center.z);

    qDebug("points size=%d",points.size());
    xozPlane.clear();
    yozPlane.clear();
    xoyPlane.clear();
    for(int i=0;i<points.size();i++)
    {
        qDebug("i=%d",i);
        qDebug("points %f %f %f",points[i].x,points[i].y,points[i].z);
        qDebug("xoznormal %f %f %f",xozNormal.x,xozNormal.y,xozNormal.z);
        qDebug("center_ %f %f %f",center_.x,center_.y,center_.z);
        xozPlane.push_back(getProjectPoint(xozNormal,center_,points[i]));
        xoyPlane.push_back(getProjectPoint(xoyNormal,center_,points[i]));
        yozPlane.push_back(getProjectPoint(yozNormal,center_,points[i]));
    }
    for(int i=0;i<points.size();i++)
    {
        xozArea+=countTriangleArea(xozPlane[i],xozPlane[(i+1)%points.size()],center_);
        xoyArea+=countTriangleArea(xoyPlane[i],xoyPlane[(i+1)%points.size()],center_);
        yozArea+=countTriangleArea(yozPlane[i],yozPlane[(i+1)%points.size()],center_);
    }
    qDebug("xoz方向: %f ",xozArea);
    qDebug("yoz方向: %f ",yozArea);
    qDebug("xoy方向: %f ",xoyArea);
}

void
Ranging::drawLines()
{
    if(screenPositions.size()<2)
        return;
    if(isClosed)
    {
        for(int i=0;i<screenPositions.size()-2;i++)
        {
            drawLine(screenPositions[i],screenPositions[i+1]);
        }
        drawLine(screenPositions[screenPositions.size()-1],screenPositions[0]);
    }
    else
    {
        for(int i=0;i<screenPositions.size()-1;i++)
        {
            drawLine(screenPositions[i],screenPositions[i+1]);
        }
    }
}

void
Ranging::calculatePerimeter()
{
    perimeter=0;
    if(isClosed)
    {
        for(int i=0;i<points.size();i++)
        {
            perimeter+=getDistance(points[i%points.size()],points[(i+1)%points.size()]);
        }
    }
    else
    {
        for(int i=0;i<points.size()-1;i++)
        {
            perimeter+= getDistance(points[i],points[i+1]);
        }
    }
}



void
Ranging::update()
{
    calculateArea();
    calculatePerimeter();
    drawLines();
}
//计算包围盒
void
Ranging::calculateobb()
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    for(int i=0;i<cloud_ptr_->getInternalCloud().size();i++)
//    {
//        pcl::PointXYZ point;
//        point.x=cloud_ptr_->getInternalCloud()[i].x;
//        point.y=cloud_ptr_->getInternalCloud()[i].y;
//        point.z=cloud_ptr_->getInternalCloud()[i].z;
//        cloud->points.push_back(point);
//    }
    qDebug("创造包围盒完毕");
    feature_extractor.setInputCloud (cloud);
    qDebug("mass");
    qDebug("mass1");
    qDebug("mass2");
    feature_extractor.compute ();
    qDebug("计算");

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    (mass_center (0), mass_center (1), mass_center (2));
    center= Point(mass_center (0), mass_center (1), mass_center (2));
    x_axis =Point(major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    y_axis=Point(middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    z_axis=Point(minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
    qDebug("??????");
}

Point
Ranging::calculateNormal(Point p1, Point p2, Point o)
{
    Point point;
    point.x = ( (p2.y-p1.y)*(o.z-p1.z)-(p2.z-p1.z)*(o.y-p1.y) );

    point.y = ( (p2.z-p1.z)*(o.x-p1.x)-(p2.x-p1.x)*(o.z-p1.z) );

    point.z = ( (p2.x-p1.x)*(o.y-p1.y)-(p2.y-p1.y)*(o.x-p1.x) );
    return point;

}

Point
Ranging::calculateCrossProduct(Point p1, Point p2)
{
    Point point;
    point.x=p1.y*p2.z-p1.z*p2.y;
    point.y=p1.z*p2.x-p1.x*p2.z;
    point.z=p1.x*p2.y-p2.x*p1.y;
    return point;
}

Point
Ranging::getProjectPoint(Point normal, Point original, Point p)
{
    qDebug("计算面积");
    float t=((normal.x*original.x+normal.y*original.y+normal.z*original.z)-(normal.x*p.x+normal.y*p.y+normal.z*p.z))/(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
    Point projectionPoint;
    projectionPoint.x= p.x+normal.x*t;
    projectionPoint.y= p.y+normal.y*t;
    projectionPoint.z= p.z+normal.z*t;
    qDebug("映射位置 %f %f %f",projectionPoint.x,projectionPoint.y,projectionPoint.z);
    return projectionPoint;
}

void
Ranging::whichPlane()//只有当图形闭合的时候才判断在哪个平面上,但是首先要计算在各个轴上的投影的面积,当投影面积最大的时候,才能判断在哪个平面上
{
    if(!isClosed)
        return;
    //TODO:计算面积,并确定在哪个平面上.
    qDebug("rangingwhichplane TODO");
}

