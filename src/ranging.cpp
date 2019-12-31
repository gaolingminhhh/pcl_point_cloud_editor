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
   // calculateobb();
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
Ranging::getPoint3D(int x, int y)
{
    Point3D point;
    int index=0;
    if(converter->getDepthValue(x,y,point,index))
    {
        points.push_back(Point(point.x,point.y,point.z));
        screenPositions.push_back(QPoint(x,y));
        indicies.push_back(index);
        highLight(index);
        times++;
    }

}

void
Ranging::highLight(int index)
{
    highLighter->highlightsinglepoint(index);
}

void
Ranging::reset()
{
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
    lines.clear();
}

float
Ranging::getDistance(Point point1, Point point2)
{
    float disx=std::abs(point1.x-point2.x);
    float disy=std::abs(point1.y-point2.y);
    float disz=std::abs(point1.z-point2.z);

    float distance=sqrt(disx*disx+disy*disy+disz*disz);
    return distance;
}

void
Ranging::end(int x, int y, BitMask modifiers, BitMask buttons)
{
    if(x!=mouse_x||y!=mouse_y)
        return;
    getPoint3D(x,y);
    update();
    //    if(times>=3)
    //        ((MainWindow*)widget)->SetArea(areaStr.append(QString::number(xoyArea)));
    //    ((MainWindow*)widget)->SetPerimeter(perimeterstr.append(QString::number(perimeter)));
}

void
Ranging::undo()
{
    points.pop_back();
    screenPositions.pop_back();
    update();
}

void Ranging::start(int x, int y, BitMask modifiers, BitMask buttons)
{
    mouse_x=x;
    mouse_y=y;
}

void
Ranging::drawLine(QPoint point1, QPoint point2)
{
    QLine line(point1.x(),point1.y(),point2.x(),point2.y());
    lines.append(line);
}

QList<QLine>
Ranging::getLines() const
{
    return lines;
}

void
Ranging::calculateArea()
{
    xozArea=0;
    yozArea=0;
    xoyArea=0;
    //当非闭合图像时,返回
    if(times<=2)
        return;
    Point center_(center.x,center.y,center.z);
    xozPlane.clear();
    yozPlane.clear();
    xoyPlane.clear();
    for(unsigned int i=0;i<points.size();i++)
    {
        xozPlane.push_back(getProjectPoint(xozNormal,center_,points[i]));
        xoyPlane.push_back(getProjectPoint(xoyNormal,center_,points[i]));
        yozPlane.push_back(getProjectPoint(yozNormal,center_,points[i]));
    }
    for(unsigned int i=0;i<points.size();i++)
    {
        xozArea+=countTriangleArea(xozPlane[i],xozPlane[(i+1)%points.size()],center_);
        xoyArea+=countTriangleArea(xoyPlane[i],xoyPlane[(i+1)%points.size()],center_);
        yozArea+=countTriangleArea(yozPlane[i],yozPlane[(i+1)%points.size()],center_);
    }
    area=xozArea;
}

void
Ranging::drawLines()
{
    if(screenPositions.size()<2)
        return;
    if(isClosed)
    {
        for(unsigned int i=0;i<screenPositions.size()-2;i++)
        {
            drawLine(screenPositions[i],screenPositions[i+1]);
        }
        drawLine(screenPositions[screenPositions.size()-1],screenPositions[0]);
    }
    else
    {
        for(unsigned int i=0;i<screenPositions.size()-1;i++)
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
        for(unsigned int i=0;i<points.size();i++)
        {
            perimeter+=getDistance(points[i%points.size()],points[(i+1)%points.size()]);
        }
    }
    else
    {
        for(unsigned int i=0;i<points.size()-1;i++)
        {
            perimeter+= getDistance(points[i],points[i+1]);
        }
    }
}



void
Ranging::update()
{
    if(times<=0)
        return;
    calculateArea();
    calculatePerimeter();
    drawLines();
}
//计算包围盒
void
Ranging::calculateobb()
{
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGBA> feature_extractor;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for(unsigned int i=0;i<cloud_ptr_->getInternalCloud().size();i++)
    {
        cloud->push_back(cloud_ptr_->getInternalCloud()[i]);
    }
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute();
    qDebug("输入包围盒");
    qDebug("计算特征");
    pcl::PointXYZRGBA min_point_OBB;
    pcl::PointXYZRGBA max_point_OBB;
    pcl::PointXYZRGBA position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);
    center= Point(mass_center (0), mass_center (1), mass_center (2));
    x_axis =Point(major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    y_axis=Point(middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    z_axis=Point(minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
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
    float t=((normal.x*original.x+normal.y*original.y+normal.z*original.z)-(normal.x*p.x+normal.y*p.y+normal.z*p.z))/(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
    Point projectionPoint;
    projectionPoint.x= p.x+normal.x*t;
    projectionPoint.y= p.y+normal.y*t;
    projectionPoint.z= p.z+normal.z*t;
    return projectionPoint;
}

void
Ranging::whichPlane()//只有当图形闭合的时候才判断在哪个平面上,但是首先要计算在各个轴上的投影的面积,当投影面积最大的时候,才能判断在哪个平面上
{
    if(!isClosed)
        return;
    //TODO:计算面积,并确定在哪个平面上.
}

void
Ranging::update(int x, int y, BitMask modifiers, BitMask buttons)
{}

void
Ranging::draw() const
{
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glColor3f(0.0,
              1.0,
              0.0);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    {
        glLoadIdentity();
        glOrtho(0, viewport[2], viewport[3], 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        {
            glLoadIdentity();
            glBegin(GL_LINES);
            {
                for(int i=0;i<lines.size();i++){

                    glVertex2d(lines[i].p1().x(),lines[i].p1().y());
                    glVertex2d(lines[i].p2().x(),lines[i].p2().y()); }
            }
            glEnd();
        }
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
    }
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    //update();
}

