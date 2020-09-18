#include <pcl/apps/point_cloud_editor/groundfilter.h>
#include <QDebug>
#include <pcl/io/pcd_io.h>


float calc_avg_point_spacing(pcl::PointCloud<pcl::PointXY>& inputCloud) {
    // Check number of points
    int point_limit = 25000;
    int iteration_limit;

    if (inputCloud.points.size() > point_limit)
        iteration_limit = point_limit;
    else
        iteration_limit = inputCloud.points.size();

    std::cout<<"iteration_limit"<<iteration_limit<< std::endl;
    // K nearest neighbor search
    // -- K = 1 gives the same point
    // -- K = 2 gives the closest neighbor
    pcl::PointCloud<pcl::PointXY>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXY>);//1
    *input_cloud_ptr=inputCloud;
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(input_cloud_ptr);

    int K = 2;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::vector<float> neighborSquaredDistance;

    for (size_t j = 0; j < iteration_limit; ++j) {
        if (kdtree.nearestKSearch(inputCloud.points[j], K, pointIdxNKNSearch,
                                  pointNKNSquaredDistance) > 0)
            neighborSquaredDistance.push_back(pointNKNSquaredDistance[1]);
    }
    float squared_avg = std::accumulate(neighborSquaredDistance.begin(),
                                        neighborSquaredDistance.end(), 0.0) /
            neighborSquaredDistance.size();

    return sqrt(squared_avg);
    //  return 0;
}


void downsample(const Cloud3D::ConstPtr& input_cloud,
                Cloud3D& downsampled_cloud,
                float leaf_size) {

    pcl::VoxelGrid<Point3D> voxel_grid;

    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setInputCloud(input_cloud);
    voxel_grid.filter(downsampled_cloud);
}


float max_elevation_diff(const Cloud3D & 	inputCloud){

    Point3D min_point, max_point;
    getMinMax3D(inputCloud, min_point, max_point);
    return max_point.z - min_point.z;
}


/* Slope estimation using ESRI's 3D Analyst Method */
float slope(Cloud3D& cloud, float cell_size) {
    // Set up minimum surface grid
    Eigen::Vector4f global_max, global_min;
    pcl::getMinMax3D<Point3D>(cloud, global_min, global_max);

    float xextent = global_max.x() - global_min.x();
    float yextent = global_max.y() - global_min.y();

    int rows = static_cast<int>(std::floor(yextent / cell_size) + 1);
    int cols = static_cast<int>(std::floor(xextent / cell_size) + 1);

    Eigen::MatrixXf A(rows, cols);
    A.setConstant(std::numeric_limits<float>::quiet_NaN());

    for (int i = 0; i < (int)cloud.points.size(); ++i) {
        // ...then test for lower points within the cell
        Point3D p = cloud.points[i];
        int row = std::floor((p.y - global_min.y()) / cell_size);
        int col = std::floor((p.x - global_min.x()) / cell_size);

        if (p.z < A(row, col) || std::isnan(A(row, col))) {
            A(row, col) = p.z;
        }
    }

    // Estimate slope
    Eigen::MatrixXf Z(rows, cols);
    Z.setConstant(std::numeric_limits<float>::quiet_NaN());

    for (int row = 0; row < rows; ++row) {
        int row_before = row - 1;
        int row_after = row + 1;
        if (row_before < 0)
            row_before = 0;
        if (row_after > rows - 1)
            row_after = rows - 1;
        for (int col = 0; col < cols; ++col) {
            int col_before = col - 1;
            int col_after = col + 1;
            if (col_before < 0)
                col_before = 0;
            if (col_after > cols - 1)
                col_after = cols - 1;

            float a = A(row_before, col_before);
            float b = A(row_before, col);
            float c = A(row_before, col_after);
            float d = A(row, col_before);
            float e = A(row, col);
            float f = A(row, col_after);
            float g = A(row_after, col_before);
            float h = A(row_after, col);
            float i = A(row_after, col_after);

            float dz_dx = ((c + 2 * f + i) - (a + 2 * d + g)) / 8 * cell_size;
            float dz_dy = ((g + 2 * h + i) - (a + 2 * b + c)) / 8 * cell_size;

            double rise_run = sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
            double slope_degrees = atan(rise_run) * 57.29578;

            if (std::isnan(Z(row, col)))
                Z(row, col) = slope_degrees;
        }
    }

    float sum = 0.0f;
    int counter = 0;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (Z(row, col) != std::numeric_limits<float>::quiet_NaN()) {
                if (Z(row, col) == Z(row, col)) {
                    sum += Z(row, col);
                    counter = counter + 1;
                }
            }
        }
    }

    return round(tan(sum / counter * PI / 180.0) * 10) / 10;
}


void upsample(
        const Cloud3D::ConstPtr& input_cloud,
        const Cloud3D::ConstPtr& downsampled_cloud,
        Cloud3D& upsampled_cloud,
        float resolution) {
    pcl::octree::OctreePointCloudSearch<Point3D> octree(resolution);
    octree.setInputCloud(input_cloud);
    octree.addPointsFromInputCloud();

    Point3D searchPoint;

    // Fill in the input point cloud data

    for (size_t i = 0; i < downsampled_cloud->points.size(); ++i) {
        searchPoint = downsampled_cloud->points[i];
        //upsampled_cloud.points.push_back(searchPoint);
        std::vector<int> pointIdxVec;
        if (octree.voxelSearch(searchPoint, pointIdxVec)) {
            for (size_t i = 0; i < pointIdxVec.size(); ++i) {
                //float elevation_diff =
                // std::abs(input_cloud->points[pointIdxVec[i]].z - searchPoint.z);
                //if (elevation_diff < 0.05)
                upsampled_cloud.points.push_back(input_cloud->points[pointIdxVec[i]]);
            }
        }
    }

    upsampled_cloud.width = upsampled_cloud.points.size();
    upsampled_cloud.height = 1;
}


void
groundFilter(Cloud3D::Ptr& inputCloud,bool isForest,bool isBuilding,Cloud3D::Ptr& groundCloud) {
    const clock_t begin_time = clock();
    // Welcome screen
    std::cout << "AUTO GROUND FILTER" << std::endl;
    std::cout << "------------------" << std::endl;

    // PCL set-up
    Cloud3D::Ptr filterCloud(
                new Cloud3D);
    Cloud3D::Ptr rotatedGround(
                new Cloud3D);

    std::cout << "Cloud before filtering: ";
    std::cout << inputCloud->points.size() << " POINTS" << std::endl;
    std::cout << "------------------" << std::endl;

    // Eigen::Vector3f before=getPlane(rotatedGround);
    //    Eigen::Vector3f after;after<<0.0f,0.0f,1.0f;
    //    Eigen::Matrix4f rotation=CreateRotateMatrix(before,after);
    //    pcl::transformPointCloud(*inputCloud,*rotatedGround,rotation);
    //    pcl::io::savePCDFileBinary("rotated.pcd",*rotatedGround);
    //    *rotatedGround=*inputCloud;

    float max_distance_ = 3.0f;
    if (isForest) {
        max_distance_ = max_elevation_diff(*inputCloud);
        std::cout << "FOREST AREA" << std::endl;
    }
    else if (isBuilding)
        std::cout << "BUILDING AREA" << std::endl;
    else {
        std::cout << "You did not enter F or B as the second argument!"<< std::endl;
        exit(1);
    }
    std::cout << "max_distance_: " << max_distance_ << std::endl;
    std::cout << "------------------" << std::endl;
    std::cout<<"Calculate average point spacing"<<std::endl;
    // Calculate average point spacing
    pcl::PointCloud<pcl::PointXY>::Ptr inputCloudXY(
                new pcl::PointCloud<pcl::PointXY>);

    inputCloudXY->points.resize(inputCloud->points.size());
    for (size_t i = 0; i < inputCloud->points.size(); i++) {
        inputCloudXY->points[i].x = inputCloud->points[i].x;
        inputCloudXY->points[i].y = inputCloud->points[i].y;
    }

    float avg_point_spacing = calc_avg_point_spacing(*inputCloudXY);
    if(avg_point_spacing<=0.02f)
    {
        avg_point_spacing=0.02f;
    }
    std::cout<<"average space "<<avg_point_spacing<<std::endl;
    // Check number of points
    bool isBig = false;
    int point_limit = BIG_DATA_POINT_LIMIT;
    float resolution = 0;




    if (inputCloud->points.size() > point_limit) {
        isBig = true;
    }
    // Downsample using octree
    if (isBig) {
        std::cout << "Downsampling..." << std::endl;
        resolution =  DOWNSAMPLE_FACTOR * avg_point_spacing;
        Cloud3D::Ptr downsampledCloud(
                    new Cloud3D);

        downsample(inputCloud, *downsampledCloud, resolution);
        //downsample(statistical_cloud, *downsampledCloud, 0.1f);

        *filterCloud = *downsampledCloud;
        std::cout << "Cloud after downsampling: ";
        std::cout << filterCloud->points.size() << " POINTS" << std::endl;
        std::cout << "------------------" << std::endl;
    } else
        *filterCloud = *inputCloud;

    // Calculate average point spacing again!
    pcl::PointCloud<pcl::PointXY>::Ptr downsampledCloudXY(
                new pcl::PointCloud<pcl::PointXY>);

    downsampledCloudXY->points.resize(filterCloud->points.size());
    for (size_t i = 0; i < filterCloud->points.size(); i++) {
        downsampledCloudXY->points[i].x = filterCloud->points[i].x;
        downsampledCloudXY->points[i].y = filterCloud->points[i].y;
    }
    avg_point_spacing = calc_avg_point_spacing(*downsampledCloudXY);
    float cell_size_ = round(avg_point_spacing * CELL_FACTOR * 100) / 100;

    std::cout << "cell_size_ = " << cell_size_ << std::endl;
    std::cout << "------------------" << std::endl;

    // Calculate average slope
    float slope_ = slope(*filterCloud, cell_size_);
    std::cout << "slope_: " << slope_ << std::endl;
    std::cout << "------------------" << std::endl;

    // Ground Filter
    std::cout << "Ground Filtering..." << std::endl;
    ground_filter(filterCloud, *groundCloud, max_distance_, cell_size_, slope_);
    std::cout << "Ground cloud after filtering: ";
    std::cout << groundCloud->points.size() << " POINTS" << std::endl;
    std::cout << "------------------" << std::endl;

    // Upsample using octree
    if (isBig) {
        std::cout << "Upsampling..." << std::endl;

        Cloud3D::Ptr upsampledCloud(
                    new Cloud3D);
        upsample(inputCloud, groundCloud, *upsampledCloud, resolution);
        // upsample(statistical_cloud, groundCloud, *upsampledCloud, 0.1f);

        std::cout << "Ground cloud after upsampling: ";
        std::cout << upsampledCloud->points.size() << " POINTS" << std::endl;

        //  *groundCloud = *upsampledCloud;
        pcl::copyPointCloud(*upsampledCloud,*groundCloud);
        std::cout << "------------------" << std::endl;
    }

    // Output

    // pcl::io::savePCDFileBinary("ground.pcd",*groundCloud);


    // Calculate execution time
    //    std::cout << "Time: " << (float(clock() - begin_time) / CLOCKS_PER_SEC / 60)
    //              << " minutes." << std::endl;

}

void ground_filter(
        const Cloud3D::ConstPtr& inputCloud,
        Cloud3D& groundCloud,
        float max_distance,
        float cell_size,
        float slope) {
    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    // Progressive Morphological Filter
    pcl::ProgressiveMorphologicalFilter<Point3D> pmf;
    pmf.setInputCloud(inputCloud);

    pmf.setMaxWindowSize(33);
    pmf.setMaxDistance(max_distance);
    pmf.setInitialDistance(0.15f);
    pmf.setCellSize(cell_size);
    pmf.setSlope(slope);
    pmf.setBase(2.0f);
    pmf.setExponential(true);

    pmf.extract(ground->indices);

    // Create the filtering object and extract the ground returns
    pcl::ExtractIndices<Point3D> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(ground);
    extract.filter(groundCloud);
}
