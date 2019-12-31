#include <pcl/apps/point_cloud_editor/interactive_panel.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/registration/icp.h>

Interactive_Panel::Interactive_Panel(CloudPtr cloud_src,
                                     CloudPtr cloud_tgt,
                                     boost::shared_ptr<HightLightPoints> highlight,
                                     QWidget *)
{
    cloud_src_ptr=cloud_src;
    cloud_tgt_ptr=cloud_tgt;
    this->highlight=highlight;
    closeBtn=new QDialogButtonBox;
    closeBtn->addButton(tr("关闭"), QDialogButtonBox::AcceptRole);
    connect(closeBtn, SIGNAL(accepted()), this, SLOT(accept()));

    checkbox=new QCheckBox("高亮显示原点云");
    connect(checkbox,SIGNAL(stateChanged(int)),this,SLOT(ChangeColor(int)));
    checkbox2=new QCheckBox("高亮显示点云");
    connect(checkbox2,SIGNAL(stateChanged(int)),this,SLOT(InverseColor(int)));
    QVBoxLayout *layout=new QVBoxLayout();
    layout->addWidget(checkbox);
    layout->addWidget(checkbox2);
    layout->addWidget(closeBtn);
    setLayout(layout);
    hightlightindicies.clear();
    cloudsize=cloud_tgt_ptr->size();
    for(unsigned int i=0;i<cloudsize;i++)
    {
        hightlightindicies.push_back(i);
    }
    for(unsigned int i=cloudsize;i<cloudsize+cloud_src_ptr->size();i++)
    {
        hightlightindicies2.push_back(i);
    }
    qDebug("hightlightindicies2 size %d",cloudsize+cloud_src_ptr->size());
    icp.setMaximumIterations (10);
    icp.setMaxCorrespondenceDistance (0.04);
    // 最大迭代次数
    icp.setMaximumIterations (50);
    // 两次变化矩阵之间的差值
    icp.setTransformationEpsilon (1e-10);
    // 均方误差
    icp.setEuclideanFitnessEpsilon (0.2);
    icp.setTransformationEpsilon(1e-6);
    icp.setMaximumIterations (100);
    icp.setTransformationEpsilon(0.0001);
    icp.setInputSource(cloud_src_ptr->getCloud3DPtr());
    icp.setInputTarget(cloud_tgt_ptr->getCloud3DPtr());
    icp.align (*(cloud_src_ptr->getCloud3DPtr()));
    Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
    Cloud3D ptr_cloud2;
    pcl::transformPointCloud(cloud_src_ptr->getInternalCloud(), ptr_cloud2, icp_trans);
    cloud_tgt_ptr->append(ptr_cloud2);
    qDebug("cloud_tgt_ptr size %d",cloud_tgt_ptr->size());
}

Interactive_Panel::~Interactive_Panel()
{
    delete checkbox;
    delete checkbox2;
    delete closeBtn;
}

void
Interactive_Panel::ChangeColor(int i)
{
    qDebug("int i %d",i);
    if(i==2)
    {
        highlight->highlightpoints(hightlightindicies);
    }
    else
    {
        highlight->dishighlight(hightlightindicies);
    }
    update();
}


void
Interactive_Panel::InverseColor(int i)
{
    qDebug("inversecolor %d",i);
    if(i==2)
    {
        highlight->highlightpoints(hightlightindicies2);
    }
    else
    {
        highlight->dishighlight(hightlightindicies2);
    }
    update();
}


void
Interactive_Panel::accept()
{
    highlight->dishighlight(hightlightindicies);
    highlight->dishighlight(hightlightindicies2);
    cloud_src_ptr->clear();
    cloud_tgt_ptr->resize(cloudsize);
    //delete cloud_out_ptr;
    this->done(0);
    qDebug("last point :%f %f %f",cloud_tgt_ptr->getInternalCloud()[cloud_tgt_ptr->size()-1].x,cloud_tgt_ptr->getInternalCloud()[cloud_tgt_ptr->size()-1].y,cloud_tgt_ptr->getInternalCloud()[cloud_tgt_ptr->size()-1].z);

}
