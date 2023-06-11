#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

std::vector<double> calculateGridSize(const std::string& pcdFilePath)
{
    // Load the .pcd file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilePath, *cloud) == -1)
    {
        std::cout << "Error loading file." << std::endl;
        return std::vector<double>{0};
    }

    // Find the minimum and maximum values of x and y coordinates
    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();

    for (const auto& point : cloud->points)
    {
        if (point.x < min_x)
            min_x = point.x;
        if (point.x > max_x)
            max_x = point.x;
        if (point.y < min_y)
            min_y = point.y;
        if (point.y > max_y)
            max_y = point.y;
    }

    // Calculate the total grid size
    double grid_size_x = max_x - min_x;
    double grid_size_y = max_y - min_y;

    std::vector<double> res(4);
    res[0]= grid_size_x ;
    res[1]= grid_size_y;
    res[2]= min_x;
    res[3]= max_x;
    res[4]= min_y;
    res[5]= max_y;
    return res;
}

int main()
{
    std::string pcdFilePath = "/home/sayon/autoware_map/town01/pointcloud_map.pcd";
    std::vector<double> res = calculateGridSize(pcdFilePath);
    std::cout<<res[0]<<std::endl;
    std::cout<<res[1]<<std::endl;
    std::cout<<res[2]<<std::endl;
    std::cout<<res[3]<<std::endl;
    std::cout<<res[4]<<std::endl;
    std::cout<<res[5]<<std::endl;
    return 0;
}

