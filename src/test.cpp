#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/apps/point_cloud_editor/test.h>


pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRGB2GRAY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gray(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_gray->height = cloud->height;
    cloud_gray->width = cloud->width;

    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++) {
        // Color conversion
        cv::Mat pixel(1, 1, CV_8UC3, cv::Scalar(it->r, it->g, it->b));
        cv::Mat temp;
        cv::cvtColor(pixel, temp, CV_RGB2GRAY);

        pcl::PointXYZI pointI;
        pointI.x = it->x;
        pointI.y = it->y;
        pointI.z = it->z;
        pointI.intensity = temp.at<uchar>(0, 0);

        cloud_gray->push_back(pointI);

    }
    return cloud_gray;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRGB2S(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_gray(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_gray->height = cloud->height;
    cloud_gray->width = cloud->width;

    for (pcl::PointCloud<pcl::PointXYZRGBA>::iterator it = cloud->begin(); it != cloud->end(); it++) {
        // Color conversion
        cv::Mat pixel(1, 1, CV_8UC3, cv::Scalar(it->r, it->g, it->b));
        cv::Mat temp;
        cv::cvtColor(pixel, temp, CV_RGB2HSV);

        pcl::PointXYZI pointI;
        pointI.x = it->x;
        pointI.y = it->y;
        pointI.z = it->z;
        pointI.intensity = temp.at<uchar>(0, 1);

        cloud_gray->push_back(pointI);

    }
    return cloud_gray;
}


//创建旋转矩阵
Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before,Eigen::Vector3f after)
{
    before.normalize();
    after.normalize();

    float angle = acos(before.dot(after));
    Eigen::Vector3f p_rotate =before.cross(after);
    p_rotate.normalize();

    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));//这里跟公式比多了一个括号，但是看实验结果它是对的。
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));


    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));


    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) +p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

    return rotationMatrix;
}

//计算平均深度值
float getZDepth(pcl::PointCloud<pcl::PointXYZRGBA>& inputCloud)
{
    //int point_limit = 25000;
    int iteration_limit=inputCloud.points.size();

    //    if (inputCloud.points.size() > point_limit)
    //        iteration_limit = point_limit;
    //    else
    //        iteration_limit = inputCloud.points.size();
    float zAccumulate=0;
    for (int j = 0; j < iteration_limit; ++j) {
        zAccumulate+=inputCloud[j].z;
    }
    float ZDepth = zAccumulate /
            inputCloud.points.size();
    std::cout<<"深度是:"<<ZDepth<<std::endl;
    return ZDepth;
}

std::vector<int> getIntensityIndicies(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_orig )
{

    std::cout << "Cloud reading failed." << std::endl;

    pcl::PointCloud <pcl::PointXYZI>::Ptr cloud = cloudRGB2S(cloud_orig);

    cv::Mat gray_values(1, cloud->size(), CV_8U);
    cv::Mat temp;

    int counter = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++) {
        gray_values.at<uchar>(0, counter) = it->intensity;
        counter++;
    }

    double thres_v = cv::threshold(gray_values, temp, 0, 255, CV_THRESH_OTSU);
    std::cout << "Otsu threshold value = " << thres_v << std::endl;
    int i=0;
    std::vector<int> indicies;
    for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++) {
        float v = it->intensity;
        if (v >= thres_v)
        {
            indicies.push_back(i);
        }
        i++;
    }
    return indicies;
}


float calc_avg_point_spacing(pcl::PointCloud<pcl::PointXY>& inputCloud) {
    // Check number of points
    int point_limit = 25000;
    int iteration_limit;

    if (inputCloud.points.size() > point_limit)
        iteration_limit = point_limit;
    else
        iteration_limit = inputCloud.points.size();

    // K nearest neighbor search
    // -- K = 1 gives the same point
    // -- K = 2 gives the closest neighbor
    pcl::PointCloud<pcl::PointXY>::Ptr input_cloud_ptr(&inputCloud);
    std::cout<<"3"<<std::endl;
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(input_cloud_ptr);
    int K = 2;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::vector<float> neighborSquaredDistance;

    for (int j = 0; j < iteration_limit; ++j) {
        if (kdtree.nearestKSearch(inputCloud.points[j], K, pointIdxNKNSearch,
                                  pointNKNSquaredDistance) > 0)
            neighborSquaredDistance.push_back(pointNKNSquaredDistance[1]);
    }
    float squared_avg = std::accumulate(neighborSquaredDistance.begin(),
                                        neighborSquaredDistance.end(), 0.0) /
            neighborSquaredDistance.size();

    return sqrt(squared_avg);
}



//max distance
float max_elevation_diff(const pcl::PointCloud<pcl::PointXYZRGBA> &inputCloud){

    pcl::PointXYZRGBA min_point, max_point;
    pcl::getMinMax3D(inputCloud, min_point, max_point);
    return max_point.z - min_point.z;
}


std::string intToString(int v)
{
    char buf[32] = {0};
    snprintf(buf, sizeof(buf), "%u", v);

    std::string str = buf;
    return str;
}


//计算颜色
pcl::PointXYZRGBA findNearestPointsZDepth(std::vector<pcl::PointXYZRGBA> &points)
{
    float x,y,z;
    x=y=z=0;
    int r,g,b,a;
    pcl::PointXYZRGBA point;

    for(int i=0;i<points.size();i++)
    {
        x+=points[i].x;
        y+=points[i].y;
        z+=points[i].z;
        r+=points[i].r;
        g+=points[i].g;
        b+=points[i].b;
        a+=points[i].a;
    }
    point.x=x/points.size();
    point.y=y/points.size();
    point.z=z/points.size();
    point.r=r/points.size();
    point.g=g/points.size();
    point.b=b/points.size();
    point.a=a/points.size();
    return point;
}

//计算点云的裂缝
void TagCrackPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,int nearestCount)
{

    //TODO 找到最近的n个点,计算平均值
    //构建kdtree
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    tree->setInputCloud(cloud);
    std::vector<int> nearestIndicies(nearestCount);
    std::vector<float> nearestDistance(nearestCount);
    std::vector<pcl::PointXYZRGBA> nearestPoints(nearestCount);
    pcl::PointXYZRGBA point;
    for(int i=0;i<cloud->points.size();i++){
        if(tree->nearestKSearch(cloud->points[i],nearestCount,nearestIndicies,nearestDistance)>0)
        {
            for(int ii=0;ii<nearestIndicies.size();ii++)
            {
                nearestPoints.push_back(cloud->points[nearestIndicies[ii]]);
            }
            point=findNearestPointsZDepth(nearestPoints);
            if(std::abs(cloud->points[i].z-point.z)>1&&(point.r+point.g+point.b-(cloud->points[i].r+cloud->points[i].g+cloud->points[i].b)>10))
            {
                cloud->points[i].r=255;
                cloud->points[i].g=0;
                cloud->points[i].b=0;
                std::cout<<i<<" is a crack point"<<std::endl;
            }
        }
    }
}
//计算深度
//float
////计算法线
//pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZRGBA> cloud_filter,int knearth)
//{
//    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
//    pcl::PointCloud<pcl::Normal>::Ptr mynormals(new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
//    tree->setInputCloud(cloud_filter);
//    ne.setInputCloud(cloud_filter);
//    ne.setSearchMethod(tree);
//    ne.setKSearch(knearth);
//    ne.compute(*mynormals);
//    return mynormals;
//}

////
//pcl::PointCloud<pcl::PointXYZRGBA> getGrowingSeg(pcl::PointCloud<pcl::Normal> Normals,int MinClusterSize,int MaxClusterSize,
//                                                 int NumberOfNeighbours,pcl::PointCloud<pcl::PointXYZRGBA> CloudFilter,
//                                                 float SmoothnessThreshold,float CurvatureThreshold)
//{
//    pcl::RegionGrowing<PointT, pcl::Normal> reg;
//    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
//    reg.setMinClusterSize(MinClusterSize);
//    reg.setMaxClusterSize(MaxClusterSize);
//    reg.setSearchMethod(tree);
//    reg.setNumberOfNeighbours(NumberOfNeighbours);
//    reg.setInputCloud(CloudFilter);
//    reg.setInputNormals(Normals);
//    reg.setSmoothnessThreshold(SmoothnessThreshold/ 180.0 * M_PI);
//    reg.setCurvatureThreshold(CurvatureThreshold);
//    std::vector <pcl::PointIndices> clusters;
//    reg.extract(clusters);
//    /* wk 添加： 可视化调试 */
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>());
//    cloud_segmented = reg.getColoredCloud();
//    return cloud_segmented;
//}
std::vector<int> getEdgeIndex(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
    //   int KNumbersNeighbor = 80; // numbers of neighbors 7 , 120
    std::vector<int> NeighborsKNSearch(KNumbersNeighborOfEdge);
    std::vector<float> NeighborsKNSquaredDistance(KNumbersNeighborOfEdge);

    int* NumbersNeighbor = new  int [cloud ->points.size ()];
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
    kdtree.setInputCloud (cloud);
    pcl::PointXYZRGBA searchPoint;

    double* Sigma = new  double [cloud->points.size()];

    //  ************ All the Points of the cloud *******************
    for (int i = 0; i < cloud ->points.size (); ++i) {

        searchPoint.x =   cloud->points[i].x;
        searchPoint.y =   cloud->points[i].y;
        searchPoint.z =   cloud->points[i].z;

        if ( kdtree.nearestKSearch (searchPoint, KNumbersNeighborOfEdge, NeighborsKNSearch, NeighborsKNSquaredDistance) > 0 ) {
            NumbersNeighbor[i]= NeighborsKNSearch.size (); }
        else { NumbersNeighbor[i] = 0; }

        float Xmean; float Ymean; float Zmean;
        float sum= 0.00;
        // Computing Covariance Matrix
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += cloud->points[ NeighborsKNSearch[ii] ].x;
        }
        Xmean = sum / NumbersNeighbor[i] ;
        sum= 0.00;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += cloud->points[NeighborsKNSearch[ii] ].y;
            //sum += cloud->points[NeighborsKNSearch[ii] ].g;
        }
        Ymean = sum / NumbersNeighbor[i] ;
        sum= 0.00;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += cloud->points[NeighborsKNSearch[ii] ].z;
            //sum += cloud->points[NeighborsKNSearch[ii] ].b;
        }
        Zmean = sum / NumbersNeighbor[i] ;

        float	CovXX;  float CovXY; float CovXZ; float CovYX; float CovYY; float CovYZ; float CovZX; float CovZY; float CovZZ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].x - Xmean )  );
            //sum += ( (cloud->points[NeighborsKNSearch[ii] ].r - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].r - Xmean )  );
        }
        CovXX = sum / ( NumbersNeighbor[i]-1) ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );
            //sum += ( (cloud->points[NeighborsKNSearch[ii] ].r - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].g - Ymean )  );
        }
        CovXY = sum / ( NumbersNeighbor[i]-1) ;

        CovYX = CovXY ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );
            //sum += ( (cloud->points[NeighborsKNSearch[ii] ].r - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].b - Zmean )  );
        }
        CovXZ= sum / ( NumbersNeighbor[i]-1) ;

        CovZX = CovXZ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );
            //sum += ( (cloud->points[NeighborsKNSearch[ii] ].g - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].g - Ymean )  );
        }
        CovYY = sum / ( NumbersNeighbor[i]-1) ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );
            //sum += ( (cloud->points[NeighborsKNSearch[ii] ].g - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].b - Zmean )  );
        }
        CovYZ = sum / ( NumbersNeighbor[i]-1) ;

        CovZY = CovYZ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].z - Zmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );
            //sum += ( (cloud->points[NeighborsKNSearch[ii] ].b - Zmean ) * ( cloud->points[NeighborsKNSearch[ii] ].b - Zmean )  );
        }
        CovZZ = sum / ( NumbersNeighbor[i]-1) ;

        // Computing Eigenvalue and EigenVector
        Eigen::Matrix3f Cov;
        Cov << CovXX, CovXY, CovXZ, CovYX, CovYY, CovYZ, CovZX, CovZY, CovZZ;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(Cov);
        if (eigensolver.info() != Eigen::Success) abort();

        double EigenValue1 = eigensolver.eigenvalues()[0];
        double EigenValue2 = eigensolver.eigenvalues()[1];
        double EigenValue3 = eigensolver.eigenvalues()[2];

        double Smallest = 0.00; double Middle = 0.00; double Largest= 0.00;
        if (EigenValue1<  EigenValue2 ) { Smallest =  EigenValue1 ; } else { Smallest = EigenValue2 ; }
        if (EigenValue3<  Smallest ) { Smallest =  EigenValue3 ; }


        if(EigenValue1 <= EigenValue2 && EigenValue1 <= EigenValue3) {
            Smallest = EigenValue1;
            if(EigenValue2 <= EigenValue3) {Middle = EigenValue2; Largest = EigenValue3;}
            else {Middle = EigenValue3; Largest = EigenValue2;}
        }

        if(EigenValue1 >= EigenValue2 && EigenValue1 >= EigenValue3)
        {
            Largest = EigenValue1;
            if(EigenValue2 <= EigenValue3) { Smallest = EigenValue2; Middle = EigenValue3; }
            else {Smallest = EigenValue3; Middle = EigenValue2;}
        }

        if ((EigenValue1 >= EigenValue2 && EigenValue1 <= EigenValue3) || (EigenValue1 <= EigenValue2 && EigenValue1 >= EigenValue3))
        {
            Middle = EigenValue1;
            if(EigenValue2 >= EigenValue3){Largest = EigenValue2; Smallest = EigenValue3;}
            else{Largest = EigenValue3; Smallest = EigenValue2;}
        }


        Sigma[i] = (Smallest) / ( Smallest+ Middle+ Largest) ;

    } // For each point of the cloud

    std::cout<< " Computing Sigma is Done! " << std::endl;
    // Color Map For the difference of the eigen values

    double MaxD=0.00 ;
    double MinD= cloud ->points.size ();
    int Ncolors=256;

    for (int i = 0; i < cloud ->points.size (); ++i) {
        if (  Sigma [i] < MinD) MinD= Sigma [i];
        if (  Sigma[i] > MaxD) MaxD = Sigma [i];
    }

    //    for(int i=0;i<cloud->points.size();i++)
    //    {
    //        if(cloud->points[i].r+cloud->points[i].g+cloud->points[i].b<100)
    //        {
    //            cloud->points[i].r=255;
    //            cloud->points[i].g=0;
    //            cloud->points[i].b=0;
    //        }
    //    }
    std::cout<< " Minimum is :" << MinD<< std::endl;
    std::cout<< " Maximum  is :" << MaxD << std::endl;

    //   *****************************************
    int Edgepoints = 0;

    std::vector<int> indexes;
    float step = ( ( MaxD -  MinD) / Ncolors ) ;
    for (int i = 0; i < cloud ->points.size (); ++i) {
        if ( Sigma [i] > ( MinD + ( 8* step) ) ) {  //6*step

            if(cloud->points[i].r+cloud->points[i].g+cloud->points[i].b<200){
                indexes.push_back(i);
                Edgepoints ++;
            }
        }
    }
    std::cout<< " Number of Edge points  is :" << Edgepoints << std::endl;
    return indexes;

    //   *****************************************

}
std::vector<int> getColorChangedIndex(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
{
    // int KNumbersNeighbor = 80; // numbers of neighbors 7 , 120
    std::vector<int> NeighborsKNSearch(KNumbersNeighborOfColor);
    std::vector<float> NeighborsKNSquaredDistance(KNumbersNeighborOfColor);

    int* NumbersNeighbor = new  int [cloud ->points.size ()];
    pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;
    kdtree.setInputCloud (cloud);
    pcl::PointXYZRGBA searchPoint;

    double* Sigma = new  double [cloud->points.size()];

    //  ************ All the Points of the cloud *******************
    for (int i = 0; i < cloud ->points.size (); ++i) {

        searchPoint.x =   cloud->points[i].x;
        searchPoint.y =   cloud->points[i].y;
        searchPoint.z =   cloud->points[i].z;

        if ( kdtree.nearestKSearch (searchPoint, KNumbersNeighborOfColor, NeighborsKNSearch, NeighborsKNSquaredDistance) > 0 ) {
            NumbersNeighbor[i]= NeighborsKNSearch.size (); }
        else { NumbersNeighbor[i] = 0; }

        float Xmean; float Ymean; float Zmean;
        float sum= 0.00;
        // Computing Covariance Matrix
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            //   sum += cloud->points[ NeighborsKNSearch[ii] ].x;
            sum += cloud->points[ NeighborsKNSearch[ii] ].r;
        }
        Xmean = sum / NumbersNeighbor[i] ;
        sum= 0.00;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            //  sum += cloud->points[NeighborsKNSearch[ii] ].y;
            sum += cloud->points[NeighborsKNSearch[ii] ].g;
        }
        Ymean = sum / NumbersNeighbor[i] ;
        sum= 0.00;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            //    sum += cloud->points[NeighborsKNSearch[ii] ].z;
            sum += cloud->points[NeighborsKNSearch[ii] ].b;
        }
        Zmean = sum / NumbersNeighbor[i] ;

        float	CovXX;  float CovXY; float CovXZ; float CovYX; float CovYY; float CovYZ; float CovZX; float CovZY; float CovZZ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            //      sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].x - Xmean )  );
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].r - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].r - Xmean )  );
        }
        CovXX = sum / ( NumbersNeighbor[i]-1) ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            // sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].r - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].g - Ymean )  );
        }
        CovXY = sum / ( NumbersNeighbor[i]-1) ;

        CovYX = CovXY ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            //      sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].r - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].b - Zmean )  );
        }
        CovXZ= sum / ( NumbersNeighbor[i]-1) ;

        CovZX = CovXZ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            //   sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].g - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].g - Ymean )  );
        }
        CovYY = sum / ( NumbersNeighbor[i]-1) ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            //           sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].g - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].b - Zmean )  );
        }
        CovYZ = sum / ( NumbersNeighbor[i]-1) ;

        CovZY = CovYZ;

        sum = 0.00 ;
        for (int ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            //         sum += ( (cloud->points[NeighborsKNSearch[ii] ].z - Zmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );
            sum += ( (cloud->points[NeighborsKNSearch[ii] ].b - Zmean ) * ( cloud->points[NeighborsKNSearch[ii] ].b - Zmean )  );
        }
        CovZZ = sum / ( NumbersNeighbor[i]-1) ;

        // Computing Eigenvalue and EigenVector
        Eigen::Matrix3f Cov;
        Cov << CovXX, CovXY, CovXZ, CovYX, CovYY, CovYZ, CovZX, CovZY, CovZZ;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(Cov);
        if (eigensolver.info() != Eigen::Success) abort();

        double EigenValue1 = eigensolver.eigenvalues()[0];
        double EigenValue2 = eigensolver.eigenvalues()[1];
        double EigenValue3 = eigensolver.eigenvalues()[2];

        double Smallest = 0.00; double Middle = 0.00; double Largest= 0.00;
        if (EigenValue1<  EigenValue2 ) { Smallest =  EigenValue1 ; } else { Smallest = EigenValue2 ; }
        if (EigenValue3<  Smallest ) { Smallest =  EigenValue3 ; }


        if(EigenValue1 <= EigenValue2 && EigenValue1 <= EigenValue3) {
            Smallest = EigenValue1;
            if(EigenValue2 <= EigenValue3) {Middle = EigenValue2; Largest = EigenValue3;}
            else {Middle = EigenValue3; Largest = EigenValue2;}
        }

        if(EigenValue1 >= EigenValue2 && EigenValue1 >= EigenValue3)
        {
            Largest = EigenValue1;
            if(EigenValue2 <= EigenValue3) { Smallest = EigenValue2; Middle = EigenValue3; }
            else {Smallest = EigenValue3; Middle = EigenValue2;}
        }

        if ((EigenValue1 >= EigenValue2 && EigenValue1 <= EigenValue3) || (EigenValue1 <= EigenValue2 && EigenValue1 >= EigenValue3))
        {
            Middle = EigenValue1;
            if(EigenValue2 >= EigenValue3){Largest = EigenValue2; Smallest = EigenValue3;}
            else{Largest = EigenValue3; Smallest = EigenValue2;}
        }


        Sigma[i] = (Smallest) / ( Smallest+ Middle+ Largest) ;

    } // For each point of the cloud

    std::cout<< " Computing Sigma is Done! " << std::endl;
    // Color Map For the difference of the eigen values

    double MaxD=0.00 ;
    double MinD= cloud ->points.size ();
    int Ncolors=256;

    for (int i = 0; i < cloud ->points.size (); ++i) {
        if (  Sigma [i] < MinD) MinD= Sigma [i];
        if (  Sigma[i] > MaxD) MaxD = Sigma [i];
    }

    //    for(int i=0;i<cloud->points.size();i++)
    //    {
    //        if(cloud->points[i].r+cloud->points[i].g+cloud->points[i].b<100)
    //        {
    //            cloud->points[i].r=255;
    //            cloud->points[i].g=0;
    //            cloud->points[i].b=0;
    //        }
    //    }
    std::cout<< " Minimum is :" << MinD<< std::endl;
    std::cout<< " Maximum  is :" << MaxD << std::endl;

    //   *****************************************
    int Edgepoints = 0;

    std::vector<int> indexes;
    float step = ( ( MaxD -  MinD) / Ncolors ) ;
    for (int i = 0; i < cloud ->points.size (); ++i) {
        if ( Sigma [i] > ( MinD + ( 8* step) ) ) {  //6*step
            if(cloud->points[i].r+cloud->points[i].g+cloud->points[i].b<200){
                indexes.push_back(i);
                Edgepoints ++;
            }
        }
    }
    std::cout<< " Number of Color Changed points  is :" << Edgepoints << std::endl;
    return indexes;

    //   *****************************************

}




int
test()
{
    clock_t startTime,endTime;
    startTime = clock();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_statistical(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZRGBA>("yhu.pcd", *cloud);

    int pointcount=cloud->points.size();

    // 下采样
    //    std::cerr<<"下采样之前:"<<std::endl;
    //    std::cerr<<*cloud<<std::endl;
    //    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel;
    //    pcl::PointCloud<pcl::PointXY>::Ptr xy(new pcl::PointCloud<pcl::PointXY>);
    //    xy->width=cloud->width;
    //    xy->height=cloud->height;
    //    xy->points.resize(cloud->points.size());
    //    for(int i=0;i<cloud->points.size();i++)
    //    {
    //        xy->points[i].x=cloud->points[i].x;
    //        xy->points[i].y=cloud->points[i].y;
    //    }
    //    float avg=calc_avg_point_spacing(*xy);
    //    voxel.setInputCloud (cloud);
    //    voxel.setLeafSize (avg*45, avg*45, avg*45);
    //    voxel.filter (*cloud_filtered_voxel);
    //    pcl::io::savePCDFileBinary("voxel.pcd",*cloud_filtered_voxel);

    //    std::cerr<<"下采样之后:"<<std::endl;
    //    std::cerr<<*cloud<<std::endl;
    //    std::cerr<<"------------"<<endl;

    //移除离群点
    std::cerr<<"移除离群点之前:"<<std::endl;
    std::cerr<<*cloud<<endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered_statistical);
    pcl::io::savePCDFileASCII("statisticaloutliarremoval.pcd",*cloud_filtered_statistical);
    std::cerr<<"移除离群点之后:"<<std::endl;
    std::cerr<<*cloud_filtered_statistical<<std::endl;
    std::cerr<<"---------------------"<<endl;

    // 提取出平面
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.5);

    cerr<<"提取出索引"<<endl;
    cerr<<"-------------------"<<endl;
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    int nr_points=cloud_filtered_statistical->points.size();
    int ii=0;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> Points;

    while(cloud_filtered_statistical->points.size()>0.3f*nr_points)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  cloud_f(new pcl::PointCloud<pcl::PointXYZRGBA>);

        seg.setInputCloud (cloud_filtered_statistical);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        extract.setInputCloud(cloud_filtered_statistical);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);

        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        //旋转分割出来的平面,使其z方向为0
        Eigen::Vector3f before;//(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
        Eigen::Vector3f after;//(0,0,1);
        Eigen::Matrix4f  change;
        before<<coefficients->values[0],coefficients->values[1],coefficients->values[2];
        after<<0.0f,0.0f,1.0f;
        change=CreateRotateMatrix(before,after);
        pcl::transformPointCloud(*cloud_p,*cloud_p,change);
        Points.push_back(cloud_p);
        pcl::io::savePCDFileASCII("cloud_groud"+intToString(ii)+".pcd", *cloud_p);
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered_statistical.swap(cloud_f);//更新
        //    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered_RGB(new pcl::PointCloud<pcl::PointXYZRGBA>);
        //    cloud_filtered_RGB->resize(cloud_filtered->size());
        //    cloud_filtered_RGB->is_dense = false;

        //    for (int i = 0 ; i < cloud_filtered->points.size() ; ++i)
        //    {
        //        cloud_filtered_RGB->points[i].x = cloud_filtered->points[i].x;
        //        cloud_filtered_RGB->points[i].y = cloud_filtered->points[i].y;
        //        cloud_filtered_RGB->points[i].z = cloud_filtered->points[i].z;
        //    }

        ii++;
    }

    std::cerr << "Object cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    //pcl::io::savePCDFileBinary("No_ground.pcd", *cloud_filtered);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> crackedpoints;
    std::vector<std::vector<int>> crackindicies;
    for(int i=0;i<Points.size();i++)
    {
        std::vector<int> indexOtsu(getIntensityIndicies(Points[i]));
        std::vector<int> indexedge(getEdgeIndex(Points[i]));
        std::vector<int> indexcolor(getColorChangedIndex(Points[i]));
        std::cerr<<"index edge size is "<<indexedge.size()<<std::endl;
        std::vector<int> result,temp;

        std::set_intersection(indexcolor.begin(),indexcolor.end(),indexedge.begin(),indexedge.end(),std::back_inserter(temp));
        if(temp.size()!=0){

            std::set_intersection(temp.begin(),temp.end(),indexOtsu.begin(),indexOtsu.end(),std::back_inserter(result));
            if(result.size()==0)
            {
                std::cerr<<result.size()<<std::endl;
            }
            else{
                crackindicies.push_back(result);
                crackedpoints.push_back(Points[i]);
            }
            std::cerr<<"颜色变换值:"<<indexcolor.size()<<std::endl;
            std::cerr<<"边缘变换值"<<indexedge.size()<<std::endl;
            std::cerr<<"颜色变换值^边缘变换值"<<result.size()<<std::endl;
            //crackindicies.push_back(indexedge);
        }
        else
        {
            std::cerr<<"temp 的长度为0"<<std::endl;
        }
    }
    pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
    std::cerr<<"开始显示点云"<<std::endl;
    for(int i=0;i<crackindicies.size();i++)
    {
        std::vector<int> indicies=crackindicies[i];
        std::cerr<<"crack indicies size:"<<indicies.size();//2071768
        for(int j=0;j<indicies.size();j++)
        {
            crackedpoints[i]->points[indicies[j]].r=0;
            crackedpoints[i]->points[indicies[j]].g=255;
            crackedpoints[i]->points[indicies[j]].b=0;
        }
        std::cerr<<"cracked points size is "<<crackindicies[i].size()<<std::endl;
        viewer.addPointCloud(crackedpoints[i],intToString(i));
    }
    endTime=clock();
    cout << "总共花费时间 : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;



    //    for(int i=0;i<leftPoints.size();i++)
    //    {
    //        viewer.addPointCloud(leftPoints[i], intToString(i));
    //    }

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }

    return (0);
}
