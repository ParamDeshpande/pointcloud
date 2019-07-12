#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/pcl_config.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <chrono>

#include "../include/pcl_utils.h"

using namespace std::literals;
using namespace std::literals::chrono_literals;
using namespace std::chrono_literals;
extern pcl::visualization::PCLVisualizer::Ptr viewer;

//plane 
float dist_thold = 0.01;
float max_iter = 100;

//filter
float sorMean = 5;
float stdDevMultThold = 1;

void load_pcd( std::string location, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_main){
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> (location, *cloud_main); 
}

void write_pcd(std::string location, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_main){
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ( location , *cloud_main, false);
}

void sor_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered){
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (sorMean);
  sor.setStddevMulThresh (stdDevMultThold);
  sor.setNegative (false);
  sor.filter (*cloud_filtered);
}


void extract_largest_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane){

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (max_iter);
  seg.setDistanceThreshold (dist_thold);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
  //done with segmentation
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size () == 0)
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  
    // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  
  // Get the points associated with the planar surface
  extract.filter (*cloud_plane);
  std::cout << "PointCloud representing the planar component: " <<cloud_plane->points.size () << " data points." << std::endl;

}



pcl::visualization::PCLVisualizer::Ptr viewportsVis (
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.5, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("RAW_POINTCLOUD", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1, 255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, single_color1, "sample cloud1",v1);

  int v2(0);
  viewer->createViewPort(0.5, 1, 0.5, 1.0, v2);
  viewer->setBackgroundColor (10, 10, 10, v2);
  viewer->addText("FILTERED_POINTCLOUD", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 55, 55, 55);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color2, "sample cloud2",v2);

  int v3(0);
  viewer->createViewPort(0.0, 0.5, 0.0, 0.5, v3);
  viewer->setBackgroundColor (0, 0, 0, v3);
  viewer->addText("PLANE_POINTCLOUD", 10, 10, "v3 text", v3);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud3, 125, 125, 125);
  viewer->addPointCloud<pcl::PointXYZ> (cloud3, single_color3, "sample cloud3",v3);

  int v4(0);
    viewer->createViewPort(0.5, 1.0, 0.0, 0.5, v4);
    viewer->setBackgroundColor (0, 0, 0, v4);
    viewer->addText("NEW_POINTCLOUD", 10, 10, "v4 text", v4);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>   single_color4(cloud4, 185, 185, 185);
    viewer->addPointCloud<pcl::PointXYZ> (cloud4, single_color4, "sample  cloud4",v4);


  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud3");
  viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud4");
  
  viewer->addCoordinateSystem (1.0);

  return (viewer);
}

void display_4_clouds_inf( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4){
  viewer = viewportsVis(cloud1,cloud2,cloud3,cloud4);
  while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
  }
}