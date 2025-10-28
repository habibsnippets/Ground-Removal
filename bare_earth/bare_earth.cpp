#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <ground.pcd> <objects.pcd>" << std::endl;
        return -1;
    }

    std::string inputFile = argv[1];
    std::string groundFile = argv[2];
    std::string objectsFile = argv[3];
    
    // Default parameters
    float cellSize = 1.0f;
    float maxDistance = 3.0f;
    float initialDistance = 0.5f;
    float slope = 0.5f;
    int maxWindow = 20;

    // Load input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputFile, *cloud) == -1)
    {
        std::cerr << "Couldn't read file " << inputFile << std::endl;
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " points from " << inputFile << std::endl;

    // Create the filtering object
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud);
    pmf.setCellSize(cellSize);
    pmf.setMaxDistance(maxDistance);
    pmf.setInitialDistance(initialDistance);
    pmf.setSlope(slope);
    pmf.setMaxWindowSize(maxWindow);

    pcl::PointIndicesPtr ground(new pcl::PointIndices);
    pmf.extract(ground->indices);

    std::cout << "Ground points: " << ground->indices.size() << std::endl;
    std::cout << "Object points: " << cloud->size() - ground->indices.size() << std::endl;

    // Extract ground and non-ground points
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground);

    // Save ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(false);
    extract.filter(*groundCloud);
    pcl::io::savePCDFile(groundFile, *groundCloud);
    std::cout << "Saved ground points to " << groundFile << std::endl;

    // Save object points
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectsCloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setNegative(true);
    extract.filter(*objectsCloud);
    pcl::io::savePCDFile(objectsFile, *objectsCloud);
    std::cout << "Saved object points to " << objectsFile << std::endl;

    return 0;
}