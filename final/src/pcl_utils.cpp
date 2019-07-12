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

#include "../include/pcl_utils.h"

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
