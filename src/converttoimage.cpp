#include "converttoimage.h"
cv::Mat makeImageFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string dimensionToRemove, float stepSize1, float stepSize2)
{
    pcl::PointXYZ cloudMin, cloudMax;
    pcl::getMinMax3D(*cloud, cloudMin, cloudMax);

    std::string dimen1, dimen2;
    float dimen1Max, dimen1Min, dimen2Min, dimen2Max;
    if (dimensionToRemove == "x")
    {
        dimen1 = "y";
        dimen2 = "z";
        dimen1Min = cloudMin.y;
        dimen1Max = cloudMax.y;
        dimen2Min = cloudMin.z;
        dimen2Max = cloudMax.z;
    }
    else if (dimensionToRemove == "y")
    {
        dimen1 = "x";
        dimen2 = "z";
        dimen1Min = cloudMin.x;
        dimen1Max = cloudMax.x;
        dimen2Min = cloudMin.z;
        dimen2Max = cloudMax.z;
    }
    else if (dimensionToRemove == "z")
    {
        dimen1 = "x";
        dimen2 = "y";
        dimen1Min = cloudMin.x;
        dimen1Max = cloudMax.x;
        dimen2Min = cloudMin.y;
        dimen2Max = cloudMax.y;
    }

    std::vector<std::vector<int>> pointCountGrid;
    int maxPoints = 0;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> grid;

    for (float i = dimen1Min; i < dimen1Max; i += stepSize1)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr slice = passThroughFilter1D(cloud, dimen1, i, i + stepSize1);
        grid.push_back(slice);

        std::vector<int> slicePointCount;

        for (float j = dimen2Min; j < dimen2Max; j += stepSize2)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cell = passThroughFilter1D(slice, dimen2, j, j + stepSize2);

            int gridSize = grid_cell->size();
            slicePointCount.push_back(gridSize);

            if (gridSize > maxPoints)
            {
                maxPoints = gridSize;
            }
        }
        pointCountGrid.push_back(slicePointCount);
    }

    cv::Mat mat(static_cast<int>(pointCountGrid.size()), static_cast<int>(pointCountGrid.at(0).size()), CV_8UC1);
    mat = cv::Scalar(0);

    for (int i = 0; i < mat.rows; ++i)
    {
        for (int j = 0; j < mat.cols; ++j)
        {
            int pointCount = pointCountGrid.at(i).at(j);
            float percentOfMax = (pointCount + 0.0) / (maxPoints + 0.0);
            int intensity = percentOfMax * 255;

            mat.at<uchar>(i, j) = intensity;
        }
    }

    return mat;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter1D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string field, const double low, const double high) {
    if (low > high) {
        std::cout << "Warning! Min is greater than max!\n";
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(low, high);
    pass.setFilterLimitsNegative(false); // don't remove inside
    pass.filter(*cloud_filtered);
    return cloud_filtered;
}
