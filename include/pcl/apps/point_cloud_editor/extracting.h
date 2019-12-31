#ifndef EXTRACTING_H
#define EXTRACTING_H
#include <pcl/apps/point_cloud_editor/toolInterface.h>
#include <vector>
#include <QList>
#include <QLine>
#include <QPoint>
#include <pcl/apps/point_cloud_editor/screenpointconverter.h>

class Extracting:public ToolInterface
{
public:
    void start(int x, int y, BitMask modifiers, BitMask buttons) override ;
    void update (int x, int y, BitMask modifiers, BitMask buttons) override;
    void end (int x, int y, BitMask modifiers, BitMask buttons) override;
    void draw() const override;
    Extracting(CloudPtr cloud_ptr_,bool isColored,boost::shared_ptr<Converter> converter);
    ~Extracting();
    const Cloud3D& getInternalCloud();
private:
    ///选区内的点的集合
    Cloud3D cloud;
    ///屏幕坐标点
    std::vector<QPoint> screenPoints;
    ///生成的屏幕上的线
    QList<QLine> lines;
    CloudPtr cloud_ptr_;
    ///判断是否在选择框里面
    bool isInSelectBox(QPoint point);
    void createLine(QPoint p1,QPoint p2);
    void createLines();
    ///检测点的位置
    void checkPoints();
    int start_x=-1;
    int start_y=-1;
    bool isColored;
    boost::shared_ptr<Converter> converter;

};

#endif // EXTRACTING_H
