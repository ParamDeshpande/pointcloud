#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "../include/pcl_utils.h"

std::string raw_file_path = "../data/stadium_full.pcd"; 
std::string filt_file_path = "../out/stadium_clean.pcd"; 
std::string op_file_path = "../out/stadium_full.pcd"; 

    //for the diff point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZ>);    
    pcl::PointCloud<pcl::PointXYZ>::Ptr largest_plane(new  pcl::PointCloud<pcl::PointXYZ>);
   
int main(int argc, char const *argv[]){
    
    load_pcd(raw_file_path, cloud);
    sor_filter(cloud , cloud_filtered);
    write_pcd(filt_file_path, cloud_filtered);
    extract_largest_plane(cloud_filtered, largest_plane);
    write_pcd(op_file_path, largest_plane);

    return 0;

}

