#include <pcl/apps/point_cloud_editor/interactive_panel.h>
using namespace std;
Interactive_Panel::Interactive_Panel(CloudPtr cloud_src_,
                                     CloudPtr cloud_tgt_,
                                     boost::shared_ptr<HightLightPoints> highlight,
                                     QWidget *)
{
    cloud_src_ptr=cloud_src_;
    cloud_tgt_ptr=cloud_tgt_;
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
    qDebug("hightlightindicies2 size %d",cloudsize+cloud_src_ptr->size());


    //    PointCloud::Ptr result (new PointCloud), source, target;
    //    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
    //    PointCloud::Ptr temp (new PointCloud);
    //    PointCloud::Ptr final (new PointCloud);
    //    final=cloud_tgt_ptr->getCloud3DPtr();
    //    PointCloud::Ptr init_result (new PointCloud);
    //    *init_result = *(cloud_src_ptr->getCloud3DPtr());
    //    Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity ();
    //    sac_ia_align(cloud_src_ptr->getCloud3DPtr(),cloud_tgt_ptr->getCloud3DPtr(),init_result,init_transform,max_sacia_iterations,min_correspondence_dist,max_correspondence_dist);
    //    pairAlign (cloud_src_ptr->getCloud3DPtr(), init_result, temp, pairTransform, true);
    //    pairTransform*=init_transform;

    //    pcl::transformPointCloud (*temp, *result, pairTransform);

    //    cloud_tgt_ptr->append(*result);


    //icp.setMaximumIterations (10);
    //下采样滤波
    //计算法线
    //计算FPFH
    clock_t start=clock();
    PointCloud::Ptr cloud_src_origin (new PointCloud);//原点云，待配准
    *cloud_src_origin=*(cloud_src_ptr->getCloud3DPtr());
    PointCloud::Ptr cloud_tgt_origin (new PointCloud);//目标点云
    *cloud_tgt_origin=*(cloud_tgt_ptr->getCloud3DPtr());
    PclCloudPtr cloud_src (new Cloud3D);
    //得到AABB包围盒大小
    getMaxMin(cloud_src_origin,src_max_xyz,src_min_xyz);
    getMaxMin(cloud_tgt_origin,tgt_max_xyz,tgt_min_xyz);
    length_src=getMinLength(src_max_xyz,src_min_xyz)/pieceCount;
    length_tgt=getMinLength(tgt_max_xyz,tgt_min_xyz)/pieceCount;

    qDebug()<<"源点云的边界大小:"<<length_src;
    qDebug()<<"目标点云的边界大小:"<<length_tgt;

    cloudsize=cloud_src_origin->size();
    int cloudtgtsize=cloud_tgt_origin->size();
 //   voxelFilter(cloud_src_origin,cloud_src,cloudsize*1.0/100000000);
    voxelFilter(cloud_src_origin,cloud_src,length_src);

    qDebug()<<"下采样点大小"<<cloudtgtsize;
    qDebug()<<"down size *cloud_src_origin from "<<cloud_src_origin->size()<<"to"<<cloud_src->size();
    //pcl::io::savePCDFileASCII("monkey_src_down.pcd",*cloud_src);   //计算表面法线
    pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
    cloud_src_normals=getNormals(cloud_src,0.02);

    PointCloud::Ptr cloud_tgt (new PointCloud);
  //  voxelFilter(cloud_tgt_origin,cloud_tgt,cloudtgtsize*1.0/50000000);
    voxelFilter(cloud_tgt_origin,cloud_tgt,length_tgt);
    qDebug()<<"下采样点大小"<<cloudtgtsize;
    qDebug()<<"down size *cloud_tgt_origin.pcd from "<<cloud_tgt_origin->size()<<"to"<<cloud_tgt->size();
    //pcl::io::savePCDFileASCII("monkey_tgt_down.pcd",*cloud_tgt);   pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne_tgt;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
    cloud_tgt_normals=getNormals(cloud_tgt,0.02);

    //计算FPFH
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfhs_src=getFeatures(cloud_src,cloud_src_normals,0.05);
qDebug()<<"fpfhs_src size "<<fpfhs_src->size();
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfhs_tgt=getFeatures(cloud_tgt,cloud_tgt_normals,0.05);
    qDebug()<<"fpfhs_tgt size "<<fpfhs_tgt->size();
    qDebug()<<"compute *cloud_tgt fpfh";


    //SAC配准
    pcl::SampleConsensusInitialAlignment<Point3D, Point3D, pcl::FPFHSignature33> scia;
    scia.setInputSource(cloud_src);
    scia.setInputTarget(cloud_tgt);
    scia.setSourceFeatures(fpfhs_src);
    scia.setTargetFeatures(fpfhs_tgt);
    //scia.setMinSampleDistance(1);
    //scia.setNumberOfSamples(2);
    scia.setCorrespondenceRandomness(20);
    PointCloud::Ptr sac_result (new PointCloud);
    scia.align(*sac_result);
    qDebug()<<"sac has converged:"<<scia.hasConverged()<<"  score: "<<scia.getFitnessScore()<<endl;
    Eigen::Matrix4f sac_trans;
    sac_trans=scia.getFinalTransformation();
    //pcl::io::savePCDFileASCII("monkey_transformed_sac.pcd",*sac_result);
    clock_t sac_time=clock();

    //icp配准
    PointCloud::Ptr icp_result (new PointCloud);
    pcl::IterativeClosestPoint<Point3D, Point3D> icp;
    icp.setInputSource(sac_result);
    //icp.setInputSource(cloud_src_origin);
    icp.setInputTarget(cloud_tgt_origin);
    icp.setMaxCorrespondenceDistance (0.04);
    // 最大迭代次数
    icp.setMaximumIterations (100);
    // 两次变化矩阵之间的差值
    icp.setTransformationEpsilon (1e-10);
    // 均方误差
    icp.setEuclideanFitnessEpsilon (0.2);
    //  icp.align(*icp_result,sac_trans);
    icp.align(*icp_result);
    clock_t end=clock();
    qDebug()<<"total time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<" s";

    qDebug()<<"sac time: "<<(double)(sac_time-start)/(double)CLOCKS_PER_SEC<<" s";
    qDebug()<<"icp time: "<<(double)(end-sac_time)/(double)CLOCKS_PER_SEC<<" s";

    qDebug() << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

    PointCloud::Ptr result (new PointCloud);
    Eigen::Matrix4f icp_trans =icp.getFinalTransformation();
    icp_trans*=sac_trans;
    for(int i=0;i<4;i++)
    {
        qDebug()<<icp_trans(i,0)<<" "<<icp_trans(i,1)<<" "<<icp_trans(i,2)<<" "<<icp_trans(i,3);
    }


    pcl::transformPointCloud(*cloud_src_origin,*result,icp_trans);

    //    PointCloud::Ptr result (new PointCloud);
    //    PointCloud::Ptr temp (new PointCloud);
    //    PointCloud::Ptr init_result (new PointCloud);
    //    *init_result = *(cloud_tgt_ptr->getCloud3DPtr());
    //    Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity ();
    //    Eigen::Matrix4f pairTransform = Eigen::Matrix4f::Identity ();
    //    sac_ia_align(cloud_src_ptr->getCloud3DPtr(),cloud_tgt_ptr->getCloud3DPtr(),init_result,init_transform,max_sacia_iterations,min_correspondence_dist,max_correspondence_dist);
    //    pairAlign (cloud_src_ptr->getCloud3DPtr(), init_result, temp, pairTransform, true);
    //    pairTransform*=init_transform;

    //将方法封装
    //  pcl::transformPointCloud (*temp, *result, pairTransform);
    //  Eigen::Matrix4f icp_trans=icp.getFinalTransformation();
    //cout<<"ransformationProbability"<<icp.getTransformationProbability()<<endl;
    //使用创建的变换对未过滤的输入点云进行变换
    //pcl::transformPointCloud(*(cloud_tgt_ptr->getCloud3DPtr()), *icp_result, icp_trans);
    qDebug()<<icp_result->size();
    cloud_tgt_ptr->append(*result);
   // cloud_tgt_ptr->append(*cloud_src_origin);
  //  cloud_tgt_ptr->append(*cloud_src_origin);
    hightlightindicies.clear();
    for(unsigned int i=0;i<cloudsize;i++)
    {
        hightlightindicies.push_back(i);
    }
    for(unsigned int i=cloudsize;i<cloudsize+result->size();i++)
    {
        hightlightindicies2.push_back(i);
    }
}

Interactive_Panel::~Interactive_Panel()
{
    delete checkbox;
    delete checkbox2;
    delete closeBtn;
}


void Interactive_Panel::voxelFilter(PointCloud::Ptr &cloud_in, PointCloud::Ptr &cloud_out, float gridsize){
    pcl::VoxelGrid<PointT> vox_grid;
    vox_grid.setLeafSize(gridsize, gridsize, gridsize);
    vox_grid.setInputCloud(cloud_in);
    vox_grid.filter(*cloud_out);
    return;
}


//计算法线
pcl::PointCloud<pcl::Normal>::Ptr Interactive_Panel::getNormals(PointCloud::Ptr cloud, double radius)
{
    pcl::PointCloud<pcl::Normal>::Ptr normalsPtr (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
    pcl::NormalEstimation<PointT,pcl::Normal> norm_est;
    norm_est.setInputCloud(cloud);
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(radius);
    norm_est.setRadiusSearch(radius);
    norm_est.compute(*normalsPtr);
    return normalsPtr;

}



//计算特征点
FPFHCloud::Ptr Interactive_Panel::getFeatures(PointCloud::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals,double radius)
{
    FPFHCloud::Ptr features (new FPFHCloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
    pcl::FPFHEstimationOMP<PointT,pcl::Normal,FPFHT> fpfh_est;
    fpfh_est.setNumberOfThreads(4);
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setKSearch(radius);
    fpfh_est.setRadiusSearch(radius);
    fpfh_est.compute(*features);
    return features;
}



//sac_ia配准
void Interactive_Panel::sac_ia_align(PointCloud::Ptr source,PointCloud::Ptr target,PointCloud::Ptr finalcloud,Eigen::Matrix4f init_transform,
                                     int max_sacia_iterations,double min_correspondence_dist,double max_correspondence_dist)
{

    vector<int> indices1;
    vector<int> indices2;
    PointCloud::Ptr sourceds (new PointCloud);
    PointCloud::Ptr targetds (new PointCloud);
    pcl::removeNaNFromPointCloud(*source,*source,indices1);
    pcl::removeNaNFromPointCloud(*target,*target,indices2);
    //降采样
    voxelFilter(source,sourceds,VOXEL_GRID_SIZE);
    voxelFilter(target,targetds,VOXEL_GRID_SIZE);
    //计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr source_normal (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normal (new pcl::PointCloud<pcl::Normal>);
    source_normal=getNormals(sourceds,radius_normal);
    target_normal=getNormals(targetds,radius_normal);
    //计算FPFH特征
    FPFHCloud::Ptr source_feature (new FPFHCloud);
    FPFHCloud::Ptr target_feature (new FPFHCloud);
    source_feature=getFeatures(sourceds,source_normal,radius_feature);
    target_feature=getFeatures(targetds,target_normal,radius_feature);

    //SAC-IA配准
    pcl::SampleConsensusInitialAlignment<PointT,PointT,FPFHT> sac_ia;
    Eigen::Matrix4f final_transformation;
    sac_ia.setInputSource(targetds);
    sac_ia.setSourceFeatures(target_feature);
    sac_ia.setInputTarget(sourceds);
    sac_ia.setTargetFeatures(source_feature);
    sac_ia.setMaximumIterations(max_sacia_iterations);
    sac_ia.setMinSampleDistance(min_correspondence_dist);
    sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);

    PclCloudPtr output (new PointCloud);
    sac_ia.align(*output);
    init_transform=sac_ia.getFinalTransformation();
    pcl::transformPointCloud(*target,*finalcloud,init_transform);
}


void Interactive_Panel::pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample)
{
    //
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }


    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);
    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon (1e-6);
    //reg.setEuclideanFitnessEpsilon(1);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (1);
    //reg.RANSACOutlierRejectionThreshold(1.5);
    reg.setMaximumIterations (1000);
    reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
    reg.setInputSource (points_with_normals_src);
    reg.setInputTarget (points_with_normals_tgt);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

    for (int i = 0; i < 5; ++i)
    {
        points_with_normals_src = reg_result;// save cloud for visualization purpose
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

    }

    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    final_transform = targetToSource;

    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

    //add the source to the transformed target
    //*output += *cloud_src;


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
    this->done(0);
}

void
Interactive_Panel::getMaxMin(PclCloudPtr cloud, float max_xyz_[], float min_xyz_[])
{
    for (std::size_t i = 1; i < cloud->size(); ++i)
    {
        for (unsigned int j = 0; j < XYZ_SIZE; ++j)
        {
            min_xyz_[j] = std::min(min_xyz_[j], cloud->points[i].data[j]);
            max_xyz_[j] = std::max(max_xyz_[j], cloud->points[i].data[j]);
        }
    }
}

float 
Interactive_Panel::getMinLength(float max_xyz[],float min_xyz[])
{
    float x=max_xyz[0]-min_xyz[0];
    float y=max_xyz[1]-min_xyz[1];
    float z=max_xyz[2]-min_xyz[2];
    float temp=std::min(x,y);
    float min=std::min(temp,z);
    return min;
}
