#pragma once

#include <qgl.h>
#include <pcl/apps/point_cloud_editor/toolInterface.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/screenpointconverter.h>
#include <QDialog>
#include <QPushButton>
#include <QLayout>
#include <QLabel>
#include <QGLWidget>
#include <vector>
#include <QList>
#include <pcl/apps/point_cloud_editor/toolInterface.h>

enum Plane
{
    XOY,
    YOZ,
    XOZ,
};

struct Point
{
public:
    Point(float X,float Y,float Z):x(X),y(Y),z(Z){}
    Point(){}
    float x;
    float y;
    float z;

};
class Ranging:public ToolInterface
{
public:
    Ranging(boost::shared_ptr<Converter> converter,CloudPtr cloud_ptr_,boost::shared_ptr<HightLightPoints> highLighter);
    ~Ranging();
    void reset();//重置
    void undo();//撤销操作
    void start(int x, int y, BitMask modifiers, BitMask buttons) override ;
    void update (int x, int y, BitMask modifiers, BitMask buttons) override;
    void end (int x, int y, BitMask modifiers, BitMask buttons) override;
    void draw() const override;

    QList<QLine> getLines() const;
private:
    bool isClosed=false;//是否闭合
    int times=0;//输入点的个数
    std::vector<QPoint> screenPositions;//屏幕位置的集合
    //float perimeter;//周长
  //  float area;//面积
    std::vector<Point> points;//点的集合
    std::vector<int> indicies;
    void drawLines();//线段
    void drawLine(QPoint point1,QPoint point2);//划线
    void calculateArea();//计算面积
    void calculatePerimeter();//计算面积
    Point calculateNormal(Point p1,Point p2,Point o);//计算法线向量方向
    CloudPtr cloud_ptr_;
    void highLight(int index);//高亮显示
    void update();//更新
    void whichPlane();//判断点在哪个位置
    float getDistance(Point point1,Point point2);//计算两个点的距离
    boost::shared_ptr<Converter> converter;
    void getPoint3D(int x,int y);
    float countTriangleArea(Point a,Point b,Point c);
    void calculateobb();//计算最小包围盒
    Point calculateCrossProduct(Point p1,Point p2);//计算叉积
    Point getProjectPoint(Point normal,Point original,Point p);//计算点p在法线和已知起始点的平面上的投影
    float calculateDotProduct(Point3D p1,Point3D p2);//计算点积
    boost::shared_ptr<HightLightPoints> highLighter;
    Point x_axis;
    Point y_axis;
    Point z_axis;
    Point center;

    Point xozNormal;
    Point xoyNormal;
    Point yozNormal;

    std::vector<Point> xozPlane;
    std::vector<Point> yozPlane;
    std::vector<Point> xoyPlane;

    float xozArea=0;
    float xoyArea=0;
    float yozArea=0;

    int mouse_x=0;
    int mouse_y=0;

    QList<QLine> lines;

};
