#include <pcl/apps/point_cloud_editor/extracting.h>
#include <gl.h>
#include <math.h>
#include <QMessageBox>
#include <QFileDialog>
#include <pcl/io/pcd_io.h>

Extracting::Extracting(CloudPtr cloud_ptr_,bool isColored,boost::shared_ptr<Converter> converter):cloud_ptr_(cloud_ptr_),
    isColored(isColored),converter(converter)
{}

Extracting::~Extracting(){}

void
Extracting::createLine(QPoint p1,QPoint p2)
{
    QLine line(p1.x(),p1.y(),p2.x(),p2.y());
    lines.append(line);
}


void
Extracting::start(int x, int y, BitMask modifiers, BitMask buttons)
{
    start_x=x;
    start_y=y;
}

void
Extracting::end(int x, int y, BitMask modifiers, BitMask buttons)
{
    if(start_x==x&&start_y==y)
    {
        screenPoints.push_back(QPoint(x,y));
        createLines();
        checkPoints();
    }
}

void
Extracting::update(int x, int y, BitMask modifiers, BitMask buttons)
{}


bool
Extracting::isInSelectBox(QPoint pt)
{
    int count=0;
    for(int i=0;i<screenPoints.size();i++)
    {
        QPoint p1=screenPoints[i];
        QPoint p2=screenPoints[(i+1)%screenPoints.size()];
        if ( p1.y() == p2.y())
            continue;

        if ( pt.y() < std::min(p1.y(), p2.y()))
            continue;

        if ( pt.y() >= std::max(p1.y(), p2.y()))
            continue;

        double x = (double)(pt.y() - p1.y()) * (double)(p2.x() - p1.x()) / (double)(p2.y() - p1.y()) + p1.x();
        if ( x > pt.x() )
            count++;

    }
    if (count % 2 == 1) {

            return true;
        }
        else {

            return false;
        }
}

const Cloud3D&
Extracting::getInternalCloud()
{
    return cloud;
}

void
Extracting::checkPoints()
{
    if(!cloud_ptr_)
        return;
    Point3DVector ptsvec;
    cloud_ptr_->getDisplaySpacePoints(ptsvec);
    for(std::size_t i = 0; i < ptsvec.size(); ++i)
    {
      Point3D pt = ptsvec[i];
      QPoint p=converter->getScreenPosValue(pt);
      if(isInSelectBox(p))
          cloud.push_back(pt);
    }
    qDebug("cloud size %d",cloud.size());
}

void
Extracting::createLines()
{
    lines.clear();
    for(int i=0;i<screenPoints.size()-1;i++)
    {
        createLine(screenPoints[i],screenPoints[i+1]);
    }
    if(screenPoints.size()<3)
        return;
    createLine(screenPoints[screenPoints.size()-1],screenPoints[0]);
}

void
Extracting::draw() const
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
                    glVertex2d(lines[i].p2().x(),lines[i].p2().y());
                }
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
