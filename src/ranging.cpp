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
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iomanip>
using namespace Eigen;


Ranging::Ranging(boost::shared_ptr<Converter> converter,CloudPtr cloud_ptr_,boost::shared_ptr<HightLightPoints> highLighter)
{
    this->converter=converter;
    this->cloud_ptr_=cloud_ptr_;
    this->highLighter=highLighter;
    qDebug("初始化完毕");
}

Ranging::~Ranging(){}

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
    area=0;
    if(points.size()<=2){
        return;
    }
    BestFitPlane bestfitplane;
    std::vector<Vector3f> lists;
    for(int i=0;i<points.size();i++)
    {
        lists.push_back(Vector3f(points[i].x,points[i].y,points[i].z));
    }
    std::pair<Vector3f,Vector3f> plane = bestfitplane.best_plane_from_points(lists);
    Vector3f center=plane.first;
    Vector3f normal=plane.second;
    Point center_point(center[0],center[1],center[2]);
    Point normal_point(normal[0],normal[1],normal[2]);
    std::vector<Point> projectPoints;
    for(int i=0;i<lists.size();i++)
    {
        Point p=Point(lists[i][0],lists[i][1],lists[i][2]);
        Point point=getProjectPoint(normal_point,center_point,p);
        projectPoints.push_back(point);
    }
    for(int i=0;i<projectPoints.size();i++)
    {
        Point p1=Point(projectPoints[i%projectPoints.size()].x-center_point.x,projectPoints[i%projectPoints.size()].y-center_point.y,
                projectPoints[i%projectPoints.size()].z-center_point.z);
        Point p2=Point(projectPoints[(i+1)%projectPoints.size()].x-center_point.x,projectPoints[(i+1)%projectPoints.size()].y-center_point.y,
                projectPoints[(i+1)%projectPoints.size()].z-center_point.z);
        area+=getAreaOfTriangle(p1,p2);
    }
    area =((float)( (int)( (area+0.005)*100)))/100;
}

void
Ranging::drawLines()
{
    if(screenPositions.size()<2)
        return;
    lines.clear();
   // if(isClosed)
   // {
        for(unsigned int i=0;i<screenPositions.size();i++)
        {
            drawLine(screenPositions[i%screenPositions.size()],screenPositions[(i+1)%screenPositions.size()]);
        }
     //   drawLine(screenPositions[screenPositions.size()-1],screenPositions[0]);
//    }
//    else
//    {
//        for(unsigned int i=0;i<screenPositions.size()-1;i++)
//        {
//            drawLine(screenPositions[i],screenPositions[i+1]);
//        }
//    }
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
    perimeter=((float)((int)((perimeter+0.005)*100 )))/100;
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

float
Ranging::getAreaOfTriangle(Point p1, Point p2)
{
    Point p=calculateCrossProduct(p1,p2);
    float triangleArea=sqrt(p1.x*p.x+p.y*p.y+p.z*p.z)/2;
    return triangleArea;
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

