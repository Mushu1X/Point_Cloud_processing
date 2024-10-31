#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <fstream>

void loadKittiBin(const std::string &bin_path, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    std::ifstream input(bin_path, std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << bin_path << std::endl;
        return;
    }

    while (input.good() && !input.eof()) {
        float data[4];
        input.read(reinterpret_cast<char *>(data), 4 * sizeof(float));
        pcl::PointXYZ point;
        point.x = data[0];
        point.y = data[1];
        point.z = data[2];
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    input.close();
}

// Preprocess 
void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);              // Number of neighbors to analyze
    sor.setStddevMulThresh(1.0);    // Distance threshold
    sor.filter(*cloud);

    // Step 2: Downsample the point cloud using a Voxel Grid Filter
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.2f, 0.2f, 0.2f); // Define voxel size in meters
    voxel_grid.filter(*cloud);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_kitti_bin_file>" << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    loadKittiBin(argv[1], cloud);

    std::cout << "Original cloud size: " << cloud->size() << " points" << std::endl;

    // Preprocess the point cloud
    preprocessPointCloud(cloud);

    std::cout << "Processed cloud size: " << cloud->size() << " points" << std::endl;

    pcl::io::savePCDFileASCII("processed_cloud.pcd", *cloud);
    std::cout << "Processed cloud saved to 'processed_cloud.pcd'" << std::endl;

    return 0;
}
