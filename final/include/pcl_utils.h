#ifndef PCL_UTILS
#define PCL_UTILS

void load_pcd( std::string , pcl::PointCloud<pcl::PointXYZ>::Ptr );
void write_pcd(std::string , pcl::PointCloud<pcl::PointXYZ>::Ptr );
void sor_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);
void extract_largest_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr ,pcl::PointCloud<pcl::PointXYZ>::Ptr );


pcl::visualization::PCLVisualizer::Ptr viewportsVis (
    pcl::PointCloud<pcl::PointXYZ>::Ptr , 
    pcl::PointCloud<pcl::PointXYZ>::Ptr ,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ,
    pcl::PointCloud<pcl::PointXYZ>::Ptr );

void display_4_clouds_inf(
    pcl::PointCloud<pcl::PointXYZ>::Ptr , 
    pcl::PointCloud<pcl::PointXYZ>::Ptr ,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ,
    pcl::PointCloud<pcl::PointXYZ>::Ptr );

#endif //

