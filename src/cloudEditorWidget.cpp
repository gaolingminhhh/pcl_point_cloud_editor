///
/// Copyright (c) 2012, Texas A&M University
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
///  * Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above
///    copyright notice, this list of conditions and the following
///    disclaimer in the documentation and/or other materials provided
///    with the distribution.
///  * Neither the name of Texas A&M University nor the names of its
///    contributors may be used to endorse or promote products derived
///    from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
/// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
/// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
/// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
/// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
/// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
///
/// The following software was written as part of a collaboration with the
/// University of South Carolina, Interdisciplinary Mathematics Institute.
///
///
/// @file   cloudEditorWidget.cpp
/// @details the implementation of class CloudEditorWidget.
/// @author  Yue Li and Matthew Hielsberg

#include <cctype>
#include <QFileDialog>
#include <QMessageBox>
#include <QMouseEvent>
#include <qgl.h>
#include <pcl/pcl_config.h>
#include <iomanip>
#include <set>
#include <thread>
#include <pcl/registration/icp.h>

#ifdef OPENGL_IS_A_FRAMEWORK
# include <OpenGL/glu.h>
#else
# include <GL/glu.h>
#endif

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/apps/point_cloud_editor/cloudEditorWidget.h>
#include <pcl/apps/point_cloud_editor/common.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/cloudTransformTool.h>
#include <pcl/apps/point_cloud_editor/selectionTransformTool.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <pcl/apps/point_cloud_editor/select1DTool.h>
#include <pcl/apps/point_cloud_editor/select2DTool.h>
#include <pcl/apps/point_cloud_editor/copyBuffer.h>
#include <pcl/apps/point_cloud_editor/copyCommand.h>
#include <pcl/apps/point_cloud_editor/pasteCommand.h>
#include <pcl/apps/point_cloud_editor/deleteCommand.h>
#include <pcl/apps/point_cloud_editor/denoiseCommand.h>
#include <pcl/apps/point_cloud_editor/cutCommand.h>
#include <pcl/apps/point_cloud_editor/mainWindow.h>
#include <pcl/apps/point_cloud_editor/highlightpoints.h>
#include <pcl/apps/point_cloud_editor/extracting.h>
#include <QVBoxLayout>
#include <pcl/apps/point_cloud_editor/interactive_panel.h>
#include <set>
#include <pcl/apps/point_cloud_editor/editpanel.h>

CloudEditorWidget::CloudEditorWidget (QWidget *parent)
    : QGLWidget(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer |
                          QGL::Rgba | QGL::StencilBuffer), parent),
      //  QOpenGLWidget(parent),
      point_size_(2.0f), selected_point_size_(14.0f),
      cam_fov_(60.0), cam_aspect_(1.0), cam_near_(0.0001), cam_far_(100.0),
      color_scheme_(COLOR_BY_PURE), is_colored_(false)
{
    setFocusPolicy(Qt::StrongFocus);
    command_queue_ptr_ = CommandQueuePtr(new CommandQueue());
    initFileLoadMap();
    initKeyMap();
    initTimer();
    // showGif();
}

void
CloudEditorWidget::editPointCloud()
{
    QString file_path = QFileDialog::getOpenFileName(this, tr("Open File"),".","point cloud data(*.pcd)");
    if (file_path.isEmpty())
        return;
    try
    {
        loadFile(file_path.toStdString(),cloud_ptr_temp,
                 pcl_cloud_ptr_temp);

    }
    catch (...)
    {
        QMessageBox::information(this, tr("Point Cloud Editor"),
                                 tr("Can not load %1.").arg(file_path));
    }
    //新建面板,里面包含两个对象,一个是原来的点云文件,一个是刚刚打开的点云文件
    QStringList list=file_path.split("/");
    obj2name=list[list.size()-1];
    EditPanel *editpanel=new EditPanel(obj1name,obj2name,this);//(QString("原始点云"),QString("新增点云"),this);
    editpanel->setCloudEditorWidget(this);
    editpanel->raise();
    editpanel->setWindowFlags(editpanel->windowFlags() | Qt::WindowStaysOnTopHint);
    editpanel->show();
    tool_ptr_edit_pointcloud =
            boost::shared_ptr<CloudTransformTool>(new CloudTransformTool(cloud_ptr_temp));

    update();
    updateGL();
}


void
CloudEditorWidget::changePtr(int i,bool isChecked)
{
    tool_ptr_edit_pointcloud=
            boost::shared_ptr<CloudTransformTool>(new CloudTransformTool(cloud_ptr_temp));

    isSelectEditPointCloud=true;

    //当前选择的状态
    if(i==0&&isChecked)
        isCheck1stPointCloud=true;
    if(i==0&&!isChecked)
        isCheck1stPointCloud=false;
    if(i==1&&isChecked)
        isCheck2ndPointCloud=true;
    if(i==1&&!isChecked)
        isCheck2ndPointCloud=false;

    //判断选择状态
    if(isCheck1stPointCloud&&!isCheck2ndPointCloud)
        checkstate=check1st;
    else if(isCheck1stPointCloud&&isCheck2ndPointCloud)
        checkstate=checkboth;
    else if(!isCheck1stPointCloud&&isCheck2ndPointCloud)
        checkstate=check2nd;
    else
        checkstate=nocheckboth;

    switch (state) {
    case Zoom:
        zoom();
        break;
    case Move:
        move();
        break;
    default:
        view();
        break;
    }
}

CloudEditorWidget::~CloudEditorWidget ()
{
}

void
CloudEditorWidget::loadFile(const std::string &filename,CloudPtr &cloud_ptr,
                            PclCloudPtr &pcl_cloud_ptr)
{
    //  std::string ext = filename.substr(filename.find_last_of('.')+1);
    std::string ext = ".pcd";

    FileLoadMap::iterator it = cloud_load_func_map_.find(ext);
    if (it != cloud_load_func_map_.end())
        (it->second)(this, filename);
    else
        loadFilePCD(filename,cloud_ptr,pcl_cloud_ptr);
}

void
CloudEditorWidget::load ()
{
    QString file_path = QFileDialog::getOpenFileName(this, tr("Open File"),".","point cloud data(*.pcd)");
    qDebug()<<file_path;
    if (file_path.isEmpty())
        return;
    try
    {
        loadFile(file_path.toStdString(),cloud_ptr_,pcl_cloud_ptr_orig);
    }
    catch (...)
    {
        QMessageBox::information(this, tr("Point Cloud Editor"),
                                 tr("Can not load %1.").arg(file_path));
    }
    initPtrs(cloud_ptr_);
    if(cloud_ptr_)
        qDebug()<<"cloud_ptr 不为空";
    else
        qDebug()<<"cloud_ptr 为空";
    QStringList list=file_path.split("/");
    obj1name=list[list.size()-1];

    qDebug()<<"this : "<<obj1name;
    update();
    updateGL();
    isEditPointCloud=true;
}

void
CloudEditorWidget::save ()
{
    if (!cloud_ptr_)
    {
        QMessageBox::information(this, tr("点云编辑器"),
                                 tr("没有点云载入."));
        return;
    }

    saveFiles(cloud_ptr_->getInternalCloud());
}

void
CloudEditorWidget::save(const Cloud3D &cloud)
{
    if(cloud.size()==0)
        return;
    saveFiles(cloud);

}

void
CloudEditorWidget::saveFiles(const Cloud3D &cloud)
{
    QString file_path = QFileDialog::getSaveFileName(this,tr("保存点云"));

    std::string file_path_std = file_path.toStdString();
    qDebug("保存成功!");
    Cloud3D cloud3d=cloud;
    if ( (file_path_std.empty()) || (cloud.size()==0) )
        return;
    if (is_colored_)
    {
        // the swapping is due to the strange alignment of r,g,b values used by PCL..
        swapRBValues(&cloud3d);
        try
        {
            pcl::io::savePCDFileBinary(file_path_std, cloud3d);

        }
        catch (...)
        {
            QMessageBox::information(this, tr("点云编辑器"),
                                     tr("无法保存%1.").arg(file_path));
        }
        swapRBValues(&cloud3d);
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ> uncolored_cloud;
        pcl::copyPointCloud(cloud, uncolored_cloud);
        try
        {
            pcl::io::savePCDFileBinary(file_path_std, uncolored_cloud);
        }
        catch (...)
        {
            QMessageBox::information(this, tr("点云编辑器"),
                                     tr("无法保存%1.").arg(file_path));
        }
    }
}

void
CloudEditorWidget::toggleBlendMode ()
{
    if (!cloud_ptr_)
        return;
    GLint blend_src = 0;
    glGetIntegerv(GL_BLEND_SRC, &blend_src);
    if (blend_src == GL_SRC_ALPHA)
        glBlendFunc( GL_ONE, GL_ZERO );
    else
        glBlendFunc( GL_SRC_ALPHA, GL_ZERO );
    update();
}


void
CloudEditorWidget::recognize_SACMODEL_CIRCLE2D()
{
    qDebug()<<"SACMODEL_CIRCLE2D";
    recognition(pcl::SACMODEL_CIRCLE2D);
}

void
CloudEditorWidget::recognize_SACMODEL_CYLINDER()
{
    qDebug()<<"SACMODEL_CYLINDER";
    recognition(pcl::SACMODEL_CYLINDER);
}


void
CloudEditorWidget::recognize_SACMODEL_PLANE()
{
    qDebug()<<"recognize_SACMODEL_PLANE";
    //  recognition(pcl::SACMODEL_PLANE);

    if(recognition_ptr->recognizePlane(0.01))
        QMessageBox::information(this, tr("点云编辑器"),
                                 tr("识别成功."));
    else
        QMessageBox::information(this, tr("点云编辑器"),
                                 tr("未能识别."));

}

void
CloudEditorWidget::recognize_SACMODEL_SPHERE()
{
    qDebug()<<"recognize_SACMODEL_SPHERE";
    recognition(pcl::SACMODEL_SPHERE);
}

void
CloudEditorWidget::recognition(int model)
{
    bool isRecognized=recognition_ptr->getIndicies(model);
    //  qDebug()<<indiciesVector.size();
    if(isRecognized)
        QMessageBox::information(this, tr("点云编辑器"),
                                 tr("识别成功."));
    else

        QMessageBox::information(this, tr("点云编辑器"),
                                 tr("未能识别."));
}


void
CloudEditorWidget::saveExtractingFile()
{

    boost::shared_ptr<Extracting> extract_ptr_ = boost::dynamic_pointer_cast<Extracting>(tool_ptr_);
    if(!extract_ptr_)
    {
        QMessageBox::information(this, tr("点云编辑器"),
                                 tr("没有抽取点云."));
        return;
    }
    save(extract_ptr_->getInternalCloud());

}

void
CloudEditorWidget::view ()
{
    if (!cloud_ptr_){
        return;
    }
    tool_ptr_ = boost::shared_ptr<CloudTransformTool>(
                new CloudTransformTool(cloud_ptr_));
    if(isSelectEditPointCloud)
        tool_ptr_edit_pointcloud = boost::shared_ptr<CloudTransformTool>(
                    new CloudTransformTool(cloud_ptr_temp));

    state=View;
}

void
CloudEditorWidget::select1D ()
{
    if (!cloud_ptr_)
        return;
    tool_ptr_ = boost::shared_ptr<Select1DTool>(new Select1DTool(selection_ptr_,
                                                                 cloud_ptr_));
    update();
}

void
CloudEditorWidget::loadqrcode()
{
    QString file_path=QFileDialog::getOpenFileName(this,tr("打开二维码/条形码"),".","图片(*.jpg *.png)");
    if(file_path.isEmpty())
        return;

    if(!qrCodeLocater_ptr)
        qrCodeLocater_ptr=boost::shared_ptr<QRCodeLocater>(new QRCodeLocater());
    std::string file_type,file_data;
    if(qrCodeLocater_ptr->locate_recognize(file_path.toStdString(),file_type,file_data))
    {
        QMessageBox::information(this,QString("点云编辑器"),tr("条码类型:%1,\n数据:%2").arg(QString::fromStdString(file_type),QString::fromStdString(file_data)));
    }
    else
    {
        QMessageBox::information(this,QString("点云编辑器"),tr("读取失败"));
    }

}

void
CloudEditorWidget::interatact()
{
    if(!cloud_ptr_)
    {
        QMessageBox::information(this, tr("点云编辑器"),
                                 tr("请先载入点云文件."));
        return;
    }
    ///a pointer to cloud being compare edit by Echo
    QString file_path = QFileDialog::getOpenFileName(this, tr("打开点云文件"),".","point cloud data(*.pcd)");
    if (file_path.isEmpty())
        return;
    try
    {
        // loadFile(file_path.toStdString());
        std::string filename=file_path.toStdString();
        PclCloudPtr pcl_cloud_ptr;
        Cloud3D tmp;
        if (pcl::io::loadPCDFile<Point3D>(filename, tmp) == -1)
            throw;
        pcl_cloud_ptr = PclCloudPtr(new Cloud3D(tmp));
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*pcl_cloud_ptr, *pcl_cloud_ptr, index);
        Statistics::clear();
        CloudPtr cloud_ptr_temp = CloudPtr(new Cloud(*pcl_cloud_ptr, true));
        if (is_colored_)
        {
            color_scheme_ = COLOR_BY_RGB;
        }
        else
        {
            color_scheme_ = COLOR_BY_Z;
        }
        panel=new Interactive_Panel(cloud_ptr_temp,cloud_ptr_,highlight);
        panel->raise();
        panel->show();
    }
    catch (...)
    {
        QMessageBox::information(this, tr("Point Cloud Editor"),
                                 tr("Can not load %1.").arg(file_path));
    }
    update();
    updateGL();
}

void
CloudEditorWidget::select2D ()
{
    if (!cloud_ptr_)
        return;
    tool_ptr_ = boost::shared_ptr<Select2DTool>(new Select2DTool(selection_ptr_,
                                                                 cloud_ptr_));
    update();
}

void
CloudEditorWidget::select3D ()
{
    if (!cloud_ptr_)
        return;
    //tool_ptr_ = boost::shared_ptr<Select3DTool>(new Select3DTool(selection_ptr_,
    //                                                             cloud_ptr_));
    update();
}

void
CloudEditorWidget::invertSelect ()
{
    if (!selection_ptr_)
        return;
    selection_ptr_ -> invertSelect();
    cloud_ptr_->setSelection(selection_ptr_);
    update();
}

void
CloudEditorWidget::range(bool isChecked)
{
    if(!cloud_ptr_)
        return;
    rangeChecked=isChecked;
    if(isChecked){
        tool_ptr_=ranging;
    }
    else
    {
        ranging->reset();
        update();
    }
}

void
CloudEditorWidget::cancelSelect ()
{
    if (!selection_ptr_)
        return;
    selection_ptr_ -> clear();
    update();
}

void
CloudEditorWidget::copy ()
{
    if (!cloud_ptr_)
        return;
    if (!selection_ptr_ || selection_ptr_->empty())
        return;
    boost::shared_ptr<CopyCommand> c(new CopyCommand(copy_buffer_ptr_,
                                                     selection_ptr_, cloud_ptr_));
    command_queue_ptr_->execute(c);
}

void
CloudEditorWidget::paste ()
{
    if (!cloud_ptr_)
        return;
    if (!copy_buffer_ptr_ || copy_buffer_ptr_->empty())
        return;
    boost::shared_ptr<PasteCommand> c(new PasteCommand(copy_buffer_ptr_,
                                                       selection_ptr_, cloud_ptr_));
    command_queue_ptr_->execute(c);
    update();
}

void
CloudEditorWidget::remove ()
{
    if (!cloud_ptr_)
        return;
    if (!selection_ptr_ || selection_ptr_->empty())
        return;
    boost::shared_ptr<DeleteCommand> c(new DeleteCommand(selection_ptr_,
                                                         cloud_ptr_));

    command_queue_ptr_->execute(c);
    update();
}

void
CloudEditorWidget::cut ()
{
    if (!cloud_ptr_)
        return;
    if (!selection_ptr_ || selection_ptr_->empty())
        return;
    boost::shared_ptr<CutCommand> c(new CutCommand(copy_buffer_ptr_,
                                                   selection_ptr_, cloud_ptr_));
    command_queue_ptr_->execute(c);
    update();
}

void
CloudEditorWidget::transform ()
{
    if (!cloud_ptr_ || !selection_ptr_ || selection_ptr_->empty())
        return;
    tool_ptr_ = boost::shared_ptr<SelectionTransformTool>(
                new SelectionTransformTool(selection_ptr_, cloud_ptr_, command_queue_ptr_));
    update();
}
//edit by echo
void
CloudEditorWidget::zoom()
{
    if (!cloud_ptr_){
        return;}
    tool_ptr_ = boost::shared_ptr<CloudTransformTool>(
                new CloudTransformTool(cloud_ptr_));
    tool_ptr_->zoom();
    if(isSelectEditPointCloud)
        tool_ptr_edit_pointcloud->zoom();
    state=Zoom;
}

void
CloudEditorWidget::test()
{
    Cloud3D ptr_cloud2;
    //std::vector<float> v={0.885447f,0.00716902, 0.464697,-0.339696,-0.02701,0.998988,0.0360614 ,-0.0514135,-0.463965,-0.0444817,0.884739,0.0768001,0,0,0,1};
    Eigen::Matrix4f matrix;
    std::vector<float> v={1,0,0,6, 0,1,0,6, 0,0,1,100, 0,0,0,1};
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            matrix(i,j)=v[4*i+j];
        }
    }
    pcl::transformPointCloud(cloud_ptr_->getInternalCloud(),ptr_cloud2,matrix);
    swapRBValues(&ptr_cloud2);
    pcl::io::savePCDFile("/home/echo/桌面/icptestpcd/model3.pcd",ptr_cloud2);
}
void
CloudEditorWidget::move()
{
    if (!cloud_ptr_){
        return;}
    tool_ptr_ = boost::shared_ptr<CloudTransformTool>(
                new CloudTransformTool(cloud_ptr_));
    tool_ptr_->move();
    if(isSelectEditPointCloud)
        tool_ptr_edit_pointcloud->move();
    state=Move;
}


void
CloudEditorWidget::calculateBoundaries(Cloud3D& cloud_)
{
    //    std::vector<Cloud3D> segments=EuclideanSeg::segmentation(cloud_);
    //    IndexVector segmentsIndex;
    //    qDebug()<<cloud_ptr_->getCloud3DPtr()->size();
    //    PclCloudPtr cloud(new Cloud3D);
    //    segmentsIndex.push_back(0);
    //    boundaries.clear();
    //    cloud->operator +=(segments[0]);
    //    for(int i=1;i<segments.size();i++)
    //    {
    //        segmentsIndex.push_back(segmentsIndex[i-1]+segments[i-1].size());
    //        cloud->operator +=(segments[i]);
    //    }
    //    //  std::vector<std::vector<unsigned int>> boundaries;
    //    cloud_ptr_ = CloudPtr(new Cloud(*cloud, false));
    //    //  vector<unsigned int> boundaries;
    //    for(int i=0;i<segments.size();i++)
    //    {
    //  vector<unsigned int> boundariesIndex=BoundaryEstimation::getBoundary(segments[i]);
    boundaries=BoundaryEstimation::getBoundary(cloud_);
    //    for(int j=0;j<boundariesIndex.size();j++)
    //    {
    //        boundaries.push_back(segmentsIndex[i]+boundariesIndex[j]);
    //    }


    //    }
}

void
CloudEditorWidget::display_boundary(bool isChecked)
{
    if(!cloud_ptr_)
        return;

    //highlight=boost::shared_ptr<HightLightPoints>(new HightLightPoints(cloud_ptr_,selection_ptr_));
    if(isChecked)
    {
        calculateBoundaries(*(cloud_ptr_->getCloud3DPtr()));
        qDebug()<<"找到了边界点";
        highlight->highlightpoints(boundaries);
    }
    else{
        highlight->dishighlight(boundaries);
    }
    isShowBoundary=isChecked;
    qDebug()<<"is checked "<<isShowBoundary;
    update();
}

void
CloudEditorWidget::sortLine(std::vector<unsigned int> points)
{
    Point3D point1=cloud_ptr_->getCloud3DPtr()->points[points[0]];
    std::vector<int> curve;
    curve.push_back(points[0]);
    curves.push_back(curve);
    points.erase(points.begin());
    unsigned int index=0;
    while(points.size()!=0)
    {
        int next=0;
        float mindistance=getDistance(point1,cloud_ptr_->getCloud3DPtr()->points[0]);
        for(int i=1;i<points.size();i++)
        {
            if(getDistance(point1,cloud_ptr_->getCloud3DPtr()->points[points[i]])<mindistance)
            {
                next=i;
                mindistance=getDistance(point1,cloud_ptr_->getCloud3DPtr()->points[points[i]]);
            }
        }
        if(mindistance<distance)
        {
            curves[index].push_back(next);
            point1=cloud_ptr_->getCloud3DPtr()->points[points[next]];
            points.erase(points.begin()+next);
        }
        else
        {
            index++;
            point1=cloud_ptr_->getCloud3DPtr()->points[points[0]];
            std::vector<int> curven;
            curven.push_back(points[0]);
            curves.push_back(curven);
            points.erase(points.begin());
        }
    }
    qDebug()<<"points.size() "<<points.size();
    qDebug()<<"曲线数量"<<curves.size();
}


float
CloudEditorWidget::getDistance(Point3D point1, Point3D point2)
{
    float disx=std::abs(point1.x-point2.x);
    float disy=std::abs(point1.y-point2.y);
    float disz=std::abs(point1.z-point2.z);

    float distance=sqrt(disx*disx+disy*disy+disz*disz);
    return distance;
}

void
CloudEditorWidget::onMouseStopMove()
{
    if(isdisplaychecked)
    {
        displayDepthValue->getDepthValue(stop_x,stop_y,screen_pos,converter);
    }
}

void
CloudEditorWidget::displayZValue(bool isChecked)
{
    qDebug("ischecked : "+isChecked);
    if(!cloud_ptr_)
        return;
    if(isChecked){
        setMouseTracking(true);//如果被选中 开始计时
        //    displayDepthValue=boost::shared_ptr<DisplayDepthValue>(new DisplayDepthValue());
    }
    else
    {
        //  displayDepthValue.reset();
        setMouseTracking(false);
    }
    isbuttonchecked=true;
    isdisplaychecked=isChecked;
    qDebug()<<"display is checked:"<<isChecked;
    update();
}

void
CloudEditorWidget::denoise ()
{
    if (!cloud_ptr_)
        return;
    DenoiseParameterForm form;
    form.exec();
    // check for cancel.
    if (!form.ok())
    {
        return;
    }
    boost::shared_ptr<DenoiseCommand> c(new DenoiseCommand(selection_ptr_,
                                                           cloud_ptr_, form.getMeanK(), form.getStdDevThresh()));
    command_queue_ptr_->execute(c);
    update();
}

void
CloudEditorWidget::crackIdentity(bool isChecked)
{
    if(!cloud_ptr_)
        return;


    if(isChecked){
        Cloud3D::Ptr cloud_crack(new Cloud3D);
        //高亮显示
        //        CrackIdentityForm form;
        //        form.exec();
        //        // check for cancel.
        //        if (!form.ok())
        //        {
        //            return;
        //        }

        // Cloud3D::Ptr cloud_temp(new Cloud3D);
        // pcl::copyPointCloud(*cloud_ptr_->getCloud3DPtr(),*cloud_temp);
        qDebug()<<"开始进行裂缝检测";

        //  crackindicies= crackdetect(cloud_ptr_->getCloud3DPtr(),form.getknumbersneighborofcolor(), form.getknumbersneighborofedge());
        crackindicies= crackdetect(cloud_ptr_->getCloud3DPtr(),10, 10);

        qDebug()<<crackindicies.size();
        if(crackindicies.size()==0){
            QMessageBox::information(this, tr("点云编辑器"),
                                     tr("没有发现裂缝."));
        }
        else
        {
            highlight->highlightpoints(crackindicies);
        }
    }
    else{
        highlight->dishighlight(crackindicies);
    }
    update();
}

void
CloudEditorWidget::groundSeg()
{
    if(!cloud_ptr_)
        return;
    GroundFilterForm  form;
    form.exec();
    if(!form.ok())
    {
        return;
    }
    qDebug()<<"打开对话框了";

    Cloud3D::Ptr cloud_temp(new Cloud3D);
    Cloud3D::Ptr cloud_ground(new Cloud3D);
    pcl::copyPointCloud(*cloud_ptr_->getCloud3DPtr(),*cloud_temp);
    groundFilter(cloud_temp,form.getisbuilding(),form.getisforest(),cloud_ground);
    cloud_ptr_= CloudPtr(new Cloud(*cloud_ground, false));

    initPtrs(cloud_ptr_);

}

void
CloudEditorWidget::undo ()
{
    if (!cloud_ptr_)
        return;
    command_queue_ptr_ -> undo();
    update();
}

void
CloudEditorWidget::increasePointSize ()
{
    ((MainWindow*) parentWidget()) -> increaseSpinBoxValue();
    point_size_ = ((MainWindow*) parentWidget()) -> getSpinBoxValue();
    if (!cloud_ptr_)
        return;
    cloud_ptr_->setPointSize(point_size_);
    update();
}

void
CloudEditorWidget::decreasePointSize ()
{
    ((MainWindow*) parentWidget()) -> decreaseSpinBoxValue();
    point_size_ = ((MainWindow*) parentWidget()) -> getSpinBoxValue();
    if (!cloud_ptr_)
        return;
    cloud_ptr_->setPointSize(point_size_);
    update();
}

void
CloudEditorWidget::increaseSelectedPointSize ()
{
    ((MainWindow*) parentWidget()) -> increaseSelectedSpinBoxValue();
    selected_point_size_ =
            ((MainWindow*) parentWidget()) -> getSelectedSpinBoxValue();
    if (!cloud_ptr_)
        return;
    cloud_ptr_->setHighlightPointSize(selected_point_size_);
    update();
}

void
CloudEditorWidget::decreaseSelectedPointSize ()
{
    ((MainWindow*) parentWidget()) -> decreaseSelectedSpinBoxValue();
    selected_point_size_ =
            ((MainWindow*) parentWidget()) -> getSelectedSpinBoxValue();
    if (!cloud_ptr_)
        return;
    cloud_ptr_->setHighlightPointSize(selected_point_size_);
    update();
}

void
CloudEditorWidget::setPointSize (int size)
{
    point_size_ = size;
    if (!cloud_ptr_)
        return;
    cloud_ptr_->setPointSize(size);
    update();
}

void
CloudEditorWidget::showGif()
{
    QMovie *movie = new QMovie(":/giphy.gif");
    QLabel *processLabel = new QLabel(this);
    processLabel->setMovie(movie);
    movie->start();
    processLabel->hide();
}

void
CloudEditorWidget::ChangeText()
{
    if(!cloud_ptr_)
        return;
    qglColor(QColor(255,255,255));

    std::vector<unsigned int> vector;
    if(isbuttonchecked)
    {
        if(isdisplaychecked)
        {
            kdtreeSearch = boost::shared_ptr<KdtreeSearch>(new KdtreeSearch(cloud_ptr_));
            std::copy(kdtreeSearch->pointindicies.begin(),kdtreeSearch->pointindicies.end(),std::back_inserter(vector));
            highlight->highlightpoints(vector);
        }
        else
        {
            std::copy(kdtreeSearch->pointindicies.begin(),kdtreeSearch->pointindicies.end(),std::back_inserter(vector));
            highlight->dishighlight(vector);
        }
        update();
    }

    if(isdisplaychecked)
    {
        std::copy(kdtreeSearch->pointindicies.begin(),kdtreeSearch->pointindicies.end(),std::back_inserter(vector));
        for(int i=0;i<vector.size();i++)
        {
            Point3D point=cloud_ptr_->getDisplaySpacePoint(vector[i]);
            QPoint qpoint = converter->getScreenPosValue(point);
            renderText(qpoint.x(),qpoint.y(),QString::number(cloud_ptr_->getInternalCloud()[vector[i]].z));
        }
        qDebug()<<"今天周五";
    }

    isbuttonchecked=false;
}

void
CloudEditorWidget::EditCancel()
{
    isSelectEditPointCloud=false;
    checkstate=nocheckboth;
    cloud_ptr_temp.reset();
    qDebug()<<"关闭";
}

void
CloudEditorWidget::SaveEditPointCloud()
{
    Point3DVector vec;
  //  cloud_ptr_->getObjectSpacePoints(vec);
   // cloud_ptr_temp->getObjectSpacePoints(vec);
    cloud_ptr_->getDisplaySpacePoints(vec);
    cloud_ptr_temp->getDisplaySpacePoints(vec);
    Cloud3D cloud;
    cloud.width = vec.size();
    cloud.height = 1;
    for(int i=0;i<vec.size();i++)
    {
        cloud.points.push_back(vec[i]);
    }
    save(cloud);
}

void
CloudEditorWidget::setSelectedPointSize (int size)
{
    selected_point_size_ = size;
    if (!cloud_ptr_)
        return;
    cloud_ptr_ -> setHighlightPointSize(size);
    update();
}

void
CloudEditorWidget::colorByRGB ()
{
    if(is_colored_)
        color_scheme_ = COLOR_BY_RGB;
}

void
CloudEditorWidget::colorByX ()
{
    color_scheme_ = COLOR_BY_X;
}

void
CloudEditorWidget::colorByY ()
{
    color_scheme_ = COLOR_BY_Y;
}

void
CloudEditorWidget::colorByZ ()
{
    color_scheme_ = COLOR_BY_Z;
}

void
CloudEditorWidget::colorByPure ()
{
    color_scheme_ = COLOR_BY_PURE;
}

void
CloudEditorWidget::extracting()
{
    if(!cloud_ptr_)
    {
        QMessageBox::information(this, tr("点云编辑器"),
                                 tr("请先载入点云文件."));
        return;
    }
    tool_ptr_=boost::shared_ptr<Extracting>(new Extracting(cloud_ptr_,is_colored_,converter));
}

void
CloudEditorWidget::showStat ()
{
    stat_dialog_.update();
    stat_dialog_.show();
    stat_dialog_.raise();
}

void
CloudEditorWidget::initializeGL ()
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_FOG);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glEnable( GL_BLEND );
    glBlendFunc( GL_ONE, GL_ZERO );
    glHint(GL_POINT_SMOOTH_HINT & GL_LINE_SMOOTH_HINT, GL_NICEST);
    initTexture();
}

void
CloudEditorWidget::paintGL ()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    if (!cloud_ptr_)
        return;
    tool_ptr_ -> draw();;
    if(isSelectEditPointCloud){
        tool_ptr_edit_pointcloud->draw();
    }
    if (color_scheme_ == COLOR_BY_RGB)
        cloud_ptr_->drawWithRGB();
    else if (color_scheme_ == COLOR_BY_PURE)
        cloud_ptr_->drawWithHighlightColor();
    else
    {
        // Assumes that color_scheme_ contains COLOR_BY_[X,Y,Z] and the values
        // match Axis::[X,Y,Z]
        cloud_ptr_ -> setColorRampAxis(Axis(color_scheme_));
        cloud_ptr_ -> drawWithTexture();
    }
    if(cloud_ptr_temp)
    {
        cloud_ptr_temp->drawWithRGB();
    }
    //cloud_ptr_temp->drawWithRGB();
    ChangeText();
    ShowAreaAndPerimeter();
}

void CloudEditorWidget::ShowAreaAndPerimeter()
{
    if(!tool_ptr_)
        return;
    QFont font = QFont("Arial");
    font.setPointSize(20);
    if(rangeChecked){
        renderText(0,40,QString("面积:").append(QString::number( tool_ptr_->area)).append(" m2"),font);
        renderText(0,80,QString("周长:").append(QString::number( tool_ptr_->perimeter)).append("m"),font);
    }
}

void
CloudEditorWidget::resizeGL (int width, int height)
{
    glViewport(0, 0, width, height);
    cam_aspect_ = double(width) / double(height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(cam_fov_, cam_aspect_, cam_near_, cam_far_);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void
CloudEditorWidget::mousePressEvent (QMouseEvent *event)
{
    if (!tool_ptr_)
        return;
    //    tool_ptr_ -> start(event -> x(), event -> y(),
    //                       event -> modifiers(), event -> buttons());
    //    tool_ptr_edit_pointcloud -> start(event -> x(), event -> y(),
    //                                      event -> modifiers(), event -> buttons());


    if(!isSelectEditPointCloud)
    {
        tool_ptr_ -> start(event -> x(), event -> y(),
                           event -> modifiers(), event -> buttons());
    }
    switch(checkstate)
    {
    case check1st:
        tool_ptr_ -> start(event -> x(), event -> y(),
                           event -> modifiers(), event -> buttons());
        break;
    case check2nd:

        tool_ptr_edit_pointcloud -> start(event -> x(), event -> y(),
                                          event -> modifiers(), event -> buttons());
        break;
    case checkboth:
        tool_ptr_ -> start(event -> x(), event -> y(),
                           event -> modifiers(), event -> buttons());
        tool_ptr_edit_pointcloud -> start(event -> x(), event -> y(),
                                          event -> modifiers(), event -> buttons());
        break;
    case nocheckboth:
        break;
    }
    if(tool_ptr_temp)
        tool_ptr_temp -> start(event -> x(), event -> y(),
                               event -> modifiers(), event -> buttons());

    update();
    updateGL();

}

void
CloudEditorWidget::mouseMoveEvent (QMouseEvent *event)
{
    //if (!tool_ptr_&&!displayDepthValue)
    // return;
    //    if(tool_ptr_){
    //        tool_ptr_ -> update(event -> x(), event -> y(),
    //                            event -> modifiers(), event -> buttons());
    //    }
    //    if(tool_ptr_edit_pointcloud){
    //        tool_ptr_edit_pointcloud -> update(event -> x(), event -> y(),
    //                                           event -> modifiers(), event -> buttons());

    //    }

    if(tool_ptr_temp)
        tool_ptr_temp -> update(event -> x(), event -> y(),
                                event -> modifiers(), event -> buttons());

    if(!isSelectEditPointCloud)
    {
        tool_ptr_ -> update(event -> x(), event -> y(),
                            event -> modifiers(), event -> buttons());
    }
    switch(checkstate)
    {
    case check1st:
        tool_ptr_ -> update(event -> x(), event -> y(),
                            event -> modifiers(), event -> buttons());
        break;
    case check2nd:

        tool_ptr_edit_pointcloud -> update(event -> x(), event -> y(),
                                           event -> modifiers(), event -> buttons());
        break;
    case checkboth:
        tool_ptr_ -> update(event -> x(), event -> y(),
                            event -> modifiers(), event -> buttons());
        tool_ptr_edit_pointcloud -> update(event -> x(), event -> y(),
                                           event -> modifiers(), event -> buttons());
        break;
    case nocheckboth:
        break;
    }

    if(displayDepthValue)
    {
        mTimer.start(500);
        stop_x=event->x();
        stop_y=event->y();
        screen_pos=event->screenPos();
    }
    update();
    updateGL();
}


void
CloudEditorWidget::mouseReleaseEvent (QMouseEvent *event)
{
    if (!tool_ptr_)
        return;
    //    tool_ptr_ -> end(event -> x(), event -> y(),
    //                     event -> modifiers(), event -> button());
    //    if(tool_ptr_edit_pointcloud)
    //        tool_ptr_edit_pointcloud -> end(event -> x(), event -> y(),
    //                                        event -> modifiers(), event -> button());
    switch(checkstate)
    {
    case check1st:
        tool_ptr_ -> end(event -> x(), event -> y(),
                         event -> modifiers(), event -> button());
        break;
    case check2nd:

        tool_ptr_edit_pointcloud -> end(event -> x(), event -> y(),
                                        event -> modifiers(), event -> button());
        break;
    case checkboth:
        tool_ptr_ -> end(event -> x(), event -> y(),
                         event -> modifiers(), event -> button());
        tool_ptr_edit_pointcloud -> end(event -> x(), event -> y(),
                                        event -> modifiers(), event -> button());
        break;
    case nocheckboth:
        break;
    }


    if(!isSelectEditPointCloud)
    {
        tool_ptr_ -> end(event -> x(), event -> y(),
                         event -> modifiers(), event -> button());
    }

    if(tool_ptr_temp)
        tool_ptr_temp -> end(event -> x(), event -> y(),
                             event -> modifiers(), event -> buttons());

    if(highlight)
    {
        highlight->hightlight();
    }
    ((MainWindow*)parentWidget())->update();
    update();
}

void
CloudEditorWidget::initTimer()
{
    qDebug("初始化计时器!\n");
    mTimer.setSingleShot(true);
    connect(&mTimer, SIGNAL(timeout()),this,
            SLOT(onMouseStopMove()));
}

void
CloudEditorWidget::keyPressEvent (QKeyEvent *event)
{
    int key = event->key() + static_cast<int>(event->modifiers());
    std::map<int, KeyMapFunc>::iterator it = key_map_.find(key);
    if (it != key_map_.end())
    {
        (it->second)(this);
        update();
    }
}


void
CloudEditorWidget::loadFilePCD(const std::string &filename,CloudPtr &cloud,
                               PclCloudPtr &pcl_cloud_ptr)
{
    Cloud3D tmp;
    if (pcl::io::loadPCDFile<Point3D>(filename, tmp) == -1)
        throw;
    pcl_cloud_ptr = PclCloudPtr(new Cloud3D(tmp));
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*pcl_cloud_ptr, *pcl_cloud_ptr, index);
    Statistics::clear();
    cloud = CloudPtr(new Cloud(*pcl_cloud_ptr, true));

    if (isColored(filename))
    {
        swapRBValues(cloud);
        color_scheme_ = COLOR_BY_RGB;
        is_colored_ = true;
    }
    else
    {
        color_scheme_ = COLOR_BY_Z;
        is_colored_ = false;
    }
}


void
CloudEditorWidget::initPtrs(CloudPtr &cloud)
{
    selection_ptr_ = SelectionPtr(new Selection(cloud, true));
    copy_buffer_ptr_ = CopyBufferPtr(new CopyBuffer(true));
    cloud->setPointSize(point_size_);
    cloud->setHighlightPointSize(selected_point_size_);
    if(isShowBoundary){
        display_boundary(true);
    }
    highlight=boost::shared_ptr<HightLightPoints>(new HightLightPoints(cloud,selection_ptr_));
    recognition_ptr=boost::shared_ptr<Recognition>(new Recognition(cloud));
    tool_ptr_ =
            boost::shared_ptr<CloudTransformTool>(new CloudTransformTool(cloud));
    converter=boost::shared_ptr<Converter>(new Converter(cloud,highlight));
    //test();
    //  ranging=boost::shared_ptr<Ranging>(new Ranging(converter,cloud_ptr_,highlight));
    //  kdtreeSearch=boost::shared_ptr<kdtreeSearch>(new kdtreeSearch(cloud_ptr_));
    ranging=boost::shared_ptr<Ranging>(new Ranging(converter,cloud,highlight));
    displayDepthValue=boost::shared_ptr<DisplayDepthValue>(new DisplayDepthValue());

}

void
CloudEditorWidget::initFileLoadMap()
{
    cloud_load_func_map_.clear();//
    //  cloud_load_func_map_["pcd"] = &CloudEditorWidget::loadFilePCD;
}

bool
CloudEditorWidget::isColored (const std::string &fileName) const
{
    pcl::PCLPointCloud2 cloud2;
    pcl::PCDReader reader;
    reader.readHeader(fileName, cloud2);
    std::vector< pcl::PCLPointField > cloud_fields = cloud2.fields;
    for(const auto &field : cloud_fields)
    {
        std::string name(field.name);
        stringToLower(name);
        if ((name == "rgb") || (name == "rgba"))
            return true;
    }
    return false;
}

void
CloudEditorWidget::swapRBValues (CloudPtr cloud_ptr_)
{
    if (!cloud_ptr_)
        return;
    for (unsigned int i = 0; i < cloud_ptr_ -> size(); i++)
    {
        std::uint8_t cc = (*cloud_ptr_)[i].r;
        (*cloud_ptr_)[i].r = (*cloud_ptr_)[i].b;
        (*cloud_ptr_)[i].b = cc;
    }
}

void
CloudEditorWidget::swapRBValues(Cloud3D *cloud)
{
    for (unsigned int i = 0; i <  cloud->size(); i++)
    {
        std::uint8_t cc = (*cloud)[i].r;
        (*cloud)[i].r = (*cloud)[i].b;
        (*cloud)[i].b = cc;
    }
}

void
CloudEditorWidget::initKeyMap ()
{
    key_map_[Qt::Key_1] = &CloudEditorWidget::colorByPure;
    key_map_[Qt::Key_2] = &CloudEditorWidget::colorByX;
    key_map_[Qt::Key_3] = &CloudEditorWidget::colorByY;
    key_map_[Qt::Key_4] = &CloudEditorWidget::colorByZ;
    key_map_[Qt::Key_5] = &CloudEditorWidget::colorByRGB;
    key_map_[Qt::Key_C + (int) Qt::ControlModifier] = &CloudEditorWidget::copy;
    key_map_[Qt::Key_X + (int) Qt::ControlModifier] = &CloudEditorWidget::cut;
    key_map_[Qt::Key_V + (int) Qt::ControlModifier] = &CloudEditorWidget::paste;
    key_map_[Qt::Key_S] = &CloudEditorWidget::select2D;
    key_map_[Qt::Key_E] = &CloudEditorWidget::select1D;
    key_map_[Qt::Key_T] = &CloudEditorWidget::transform;
    key_map_[Qt::Key_V] = &CloudEditorWidget::view;
    key_map_[Qt::Key_Delete] = &CloudEditorWidget::remove;
    key_map_[Qt::Key_Z + (int) Qt::ControlModifier] = &CloudEditorWidget::undo;
    key_map_[Qt::Key_Equal] = &CloudEditorWidget::increasePointSize;
    key_map_[Qt::Key_Plus] = &CloudEditorWidget::increasePointSize;
    key_map_[Qt::Key_Minus] = &CloudEditorWidget::decreasePointSize;
    key_map_[Qt::Key_Equal + (int) Qt::ControlModifier] =
            &CloudEditorWidget::increaseSelectedPointSize;
    key_map_[Qt::Key_Plus + (int) Qt::ControlModifier] =
            &CloudEditorWidget::increaseSelectedPointSize;
    key_map_[Qt::Key_Minus + (int) Qt::ControlModifier] =
            &CloudEditorWidget::decreaseSelectedPointSize;
    key_map_[Qt::Key_Escape] = &CloudEditorWidget::cancelSelect;
}

void
CloudEditorWidget::initTexture ()
{
    static GLfloat colorWheel[14][3] =
    {
        {      0.0f,    0.0f,  1.0000f},
        {      0.0f, 0.2500f,  1.0000f},
        {      0.0f, 0.5000f,  1.0000f},
        {      0.0f, 0.7500f,  1.0000f},
        {      0.0f, 1.0000f,  1.0000f},
        {   0.2500f, 1.0000f,  1.0000f},
        {   0.5000f, 1.0000f,  0.7500f},
        {   0.7500f, 1.0000f,  0.5000f},
        {   1.0000f, 1.0000f,  0.2500f},
        {   1.0000f, 1.0000f,     0.0f},
        {   1.0000f, 0.7500f,     0.0f},
        {   1.0000f, 0.5000f,     0.0f},
        {   1.0000f, 0.2500f,     0.0f},
        {   1.0000f,    0.0f,     0.0f},
    };
    GLuint textures;
    glGenTextures(1,&textures);
    glBindTexture(GL_TEXTURE_1D,textures);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, 14, 0, GL_RGB , GL_FLOAT, colorWheel);
}
