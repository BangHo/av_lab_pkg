#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <cmath>
#include <vector>
#include <dynamixel_sdk.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

ros::Publisher pub, lidar_stop;
using namespace std;
using PointT = pcl::PointXYZI;



void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  for(int i = 0; i < msg->ranges.size(); i++) {
    if(msg->ranges[i] > 0.05 && msg->ranges[i] < 10.0) {
      pcl::PointXYZI point;
      point.x = msg->ranges[i] * cos(msg->angle_min + i * msg->angle_increment);
      point.y = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment);
      point.z = 0.0;
      cloud->points.push_back(point);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;

  pcl::VoxelGrid<pcl::PointXYZI> VG;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  VG.setInputCloud(cloud);
  VG.setLeafSize(0.1f, 0.1f, 0.1f);
  VG.filter(*cloud_filtered);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> cropFilter;
  cropFilter.setInputCloud(cloud_filtered);
  cropFilter.setMin(Eigen::Vector4f(-0, -2, -0, 0)); // 실험하면서 값 조정하기
  cropFilter.setMax(Eigen::Vector4f(2, 2, 0.0, 0));
  cropFilter.filter(*cloud_filtered2); 
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered2); 
  vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setInputCloud(cloud_filtered2);
  
  ec.setClusterTolerance(0.3);
  ec.setMinClusterSize(1); 
  ec.setMaxClusterSize(1000); 
  ec.setSearchMethod(tree);
  ec.extract(cluster_indices);
  int ii = 0; 
  pcl::PointCloud<PointT> TotalCloud;
  std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
  { 
    pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
    cluster->points.push_back(cloud_filtered2->points[*pit]); 
    PointT pt = cloud_filtered2->points[*pit]; 
    PointT pt2; pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z; 
    pt2.intensity = (float)(ii + 1); 
    TotalCloud.push_back(pt2);
    }
    cluster->width = cluster->size(); 
    cluster->height = 1; 
    cluster->is_dense = true; 
    clusters.push_back(cluster);
  }
    
  for (const auto& cluster : clusters) 
  {
    double x_sum = 0.0, y_sum = 0.0;
    int num_points = 0;
    bool detected = false;
    
    for (const auto& point : cluster->points) 
    {
        x_sum += point.x;
        y_sum += point.y;
        num_points++;
    }
    double x_mean = x_sum / num_points;
    double y_mean = y_sum / num_points;
    // std::cout << "x :" << x_mean << ", y :" << y_mean << '\n';

    if(x_mean>0 && x_mean<0.3000){
      if(y_mean>-0.1 && y_mean<0.1){
        std::cout << "[Object] Object!! " << std::endl; 
        detected = true;
      }
    }

    std_msgs::Bool detected_;
    detected_.data = detected;
    lidar_stop.publish(detected_);
  }

    

  pcl::PCLPointCloud2 cloud_p; 
  pcl::toPCLPointCloud2(TotalCloud, cloud_p); 
  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(cloud_p, output); 
  output.header.frame_id = "laser_frame"; 
  pub.publish(output);




}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_clustering");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, laserCallback);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_clusters", 10);
  lidar_stop = nh.advertise<std_msgs::Bool>("/Dynamic_Stop", 10);

  ros::spin();
}
