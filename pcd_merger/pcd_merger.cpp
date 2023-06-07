#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>

namespace fs = std::filesystem;

int main()
{
    // Directory containing the .pcd files to merge
    std::string directory = "/home/sayon/grider_output/";

    // Output merged point cloud file
    std::string output_file = "../merged.pcd";

    // Create a PointCloud to store the merged point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Iterate through the directory and merge .pcd files
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &entry : fs::directory_iterator(directory))
    {
        if (entry.is_regular_file())
        {
            std::string file_path = entry.path().string();
            if (fs::path(file_path).extension() == ".pcd")
            {
                if (reader.read(file_path, *cloud) == 0)
                {
                    *merged_cloud += *cloud;
                }
                else
                {
                    std::cerr << "Failed to read file: " << file_path << std::endl;
                }
            }
        }
    }

    // Save the merged point cloud
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(output_file, *merged_cloud);

    std::cout << "Merged point cloud saved to: " << output_file << std::endl;

    return 0;
}
