#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "tower_defense/ObstaclePointCloudSrv.h"
// #include "tower_defense/RandomConfigSrv.h"
// #include "tower_defense/ExtendNodeSrv.h"
// #include "tower_defense/CheckExtensionSrv.h"
// #include "tower_defense/BuildRRTSrv.h"
// #include "tower_defense/RRTPlanSrv.h"

// using tower_defense::RandomConfigSrv;
// using tower_defense::ExtendNodeSrv;
// using tower_defense::CheckExtensionSrv;
// using tower_defense::BuildRRTSrv;
// using tower_defense::RRTPlanSrv;
// using tower_defense::RRTNode;

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point;
using geometry_msgs::Point32;
using std::fabs;
using std::max;
using std::atan2;
using std::cout;
using std::vector;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

// Publisher for marker messages.
ros::Publisher markers_publisher_;

//Publisher for filtered point cloud
ros::Publisher filtered_point_cloud_publisher_;

// Publisher for 3D point clouds.
ros::Publisher point_cloud_publisher_;

//camera intrinsics
float fx = 588.446;
float fy = -564.227;
float px = 320;
float py = 240;
float a = 3.008;
float b = -0.002745;

// Markers for visualization.
Marker vertices_marker_;
Marker qrand_marker_;
Marker edges_marker_;
Marker map_marker_;
Marker plan_marker_;

// Helper function to convert ROS Point32 to Eigen Vectors.
Vector3f ConvertPointToVector(const Point32& point) {
  return Vector3f(point.x, point.y, point.z);
}

// Helper function to convert Eigen Vectors to ROS Point32.
Point32 ConvertVectorToPoint(const Vector3f& vector) {
  Point32 point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

// Return a random value between min and max.
float RandomValue(const float min, const float max) {
  const float scale = max - min;
  const float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  return (min + r * scale);
}

// Helper function to convert a 2D vector into a ros 3D point.
Point VectorToPoint(const Vector2f& v) {
  Point point;
  point.x = v.x();
  point.y = v.y();
  point.z = 0;
  return point;
}

// Helper function to visualize a point.
void DrawPoint(const Vector2f& p, Marker* marker) {
  marker->points.push_back(VectorToPoint(p));
}

// Helper function to visualize an edge.
void DrawLine(const Vector2f& p1,
              const Vector2f& p2,
              Marker* marker) {
  marker->points.push_back(VectorToPoint(p1));
  marker->points.push_back(VectorToPoint(p2));
}

// Initialize all markers.
void InitMarkers() {
  vertices_marker_.header.frame_id = "map";
  vertices_marker_.id = 1;
  vertices_marker_.type = Marker::POINTS;
  vertices_marker_.action = Marker::MODIFY;
  vertices_marker_.scale.x = 0.2;
  vertices_marker_.scale.y = 0.2;
  vertices_marker_.color.a = 1.0;
  vertices_marker_.color.r = 0.0;
  vertices_marker_.color.g = 0.0;
  vertices_marker_.color.b = 1.0;


  qrand_marker_.header.frame_id = "map";
  qrand_marker_.id = 2;
  qrand_marker_.type = Marker::POINTS;
  qrand_marker_.action = Marker::MODIFY;
  qrand_marker_.scale.x = 0.2;
  qrand_marker_.scale.y = 0.2;
  qrand_marker_.color.a = 1.0;
  qrand_marker_.color.r = 1.0;
  qrand_marker_.color.g = 0.0;
  qrand_marker_.color.b = 0.0;

  edges_marker_.header.frame_id = "map";
  edges_marker_.id = 3;
  edges_marker_.type = Marker::LINE_LIST;
  edges_marker_.action = Marker::MODIFY;
  edges_marker_.scale.x = 0.05;
  edges_marker_.scale.y = 0.05;
  edges_marker_.color.a = 1.0;
  edges_marker_.color.r = 0.0;
  edges_marker_.color.g = 1.0;
  edges_marker_.color.b = 0.0;

  map_marker_.header.frame_id = "map";
  map_marker_.id = 4;
  map_marker_.type = Marker::LINE_LIST;
  map_marker_.action = Marker::MODIFY;
  map_marker_.scale.x = 0.05;
  map_marker_.scale.y = 0.05;
  map_marker_.color.a = 1.0;
  map_marker_.color.r = 1.0;
  map_marker_.color.g = 1.0;
  map_marker_.color.b = 1.0;

  plan_marker_.header.frame_id = "map";
  plan_marker_.id = 5;
  plan_marker_.type = Marker::LINE_LIST;
  plan_marker_.action = Marker::MODIFY;
  plan_marker_.scale.x = 0.05;
  plan_marker_.scale.y = 0.05;
  plan_marker_.color.a = 1.0;
  plan_marker_.color.r = 1.0;
  plan_marker_.color.g = 0.0;
  plan_marker_.color.b = 0.0;
}

//this does not work! copied the code from previous assignment we need to use pointcloud2's everywhere or transform the pointcloud2 to pointcloud in the dpeth image call back
bool ObstaclePointCloudService(
    tower_defense::ObstaclePointCloudSrv::Request& req,
    tower_defense::ObstaclePointCloudSrv::Response& res) {
  Matrix3f R;

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      R(row, col) = req.R[col * 3 + row];
    }
  }
  const Vector3f T(req.T.x, req.T.y, req.T.z);

  vector<Vector3f> filtered_point_cloud(req.P.size());
  // Copy over all the points.
  size_t j = 0;
  vector<Vector3f> point_cloud(req.P.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(req.P[i]);
    point_cloud[i] = R * (point_cloud[i]) + T;
    if(point_cloud[i].z() > .01){
      filtered_point_cloud[j] = point_cloud[i];
      j++;
    }
  }

  filtered_point_cloud.resize(j); 
    
  // Write code here to transform the input point cloud from the Kinect reference frame to the
  // robot's reference frame. Then filter out the points corresponding to ground
  // or heights larger than the height of the robot
 
  res.P_prime.resize(j);
   
  sensor_msgs::PointCloud publish_cloud;
  publish_cloud.header.frame_id = "base_link";
  publish_cloud.points.resize(j);
  for (size_t i = 0; i < j; ++i) {
    res.P_prime[i] = ConvertVectorToPoint(filtered_point_cloud[i]);
    publish_cloud.points[i] = res.P_prime[i];
  }
  //we might want to publish res.P_prime instead because 
  filtered_point_cloud_publisher_.publish(publish_cloud);
  //printf("Hello, point cloud %d\n", (int) j);
  return true;
}
//sensor_msgs/PointCloud2.msg
void DepthImageCallback(const sensor_msgs::PointCloud2& image) {
  // Message for published 3D point clouds.
  //this works!!!
  //printf("we are reading the depth image");
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header = image.header;

  // for (unsigned int y = 0; y < image.height; ++y) {
  //   for (unsigned int x = 0; x < image.width; ++x) {
  //     uint16_t byte0 = image.data[2 * (x + y * image.width) + 0];
  //     uint16_t byte1 = image.data[2 * (x + y * image.width) + 1];
  //     if (!image.is_bigendian) {
  //       std::swap(byte0, byte1);
  //     }
  //     // Combine the two bytes to form a 16 bit value, and disregard the
  //     // most significant 4 bits to extract the lowest 12 bits.
  //     const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
  //     // Reconstruct 3D point from x, y, raw_depth.
  //     geometry_msgs::Point32 point;
  //     // Modify the following lines of code to do the right thing, with the
  //     // correct parameters.
  //     point.z = 1/(a+(b*raw_depth));
  //     point.x = ((x-px)/fx)*point.z;
  //     point.y = ((y-py)/fy)*point.z;
      point_cloud.points.push_back(point);
    }
  }
  point_cloud_publisher_.publish(point_cloud);
}

int main(int argc, char **argv) {
  InitMarkers();

  ros::init(argc, argv, "camera_rgb_optical_frame");
  ros::NodeHandle n;

  filtered_point_cloud_publisher_ = 
    n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/FilteredPointCloud", 1);

  point_cloud_publisher_ =
    n.advertise<sensor_msgs::PointCloud2>("/COMPSCI403/PointCloud", 1);

  //this works!
  ros::Subscriber depth_image_subscriber =
    n.subscribe("/camera/depth_registered/points", 1, DepthImageCallback);

  // markers_publisher_ = n.advertise<visualization_msgs::MarkerArray>(
  //     "/COMPSCI403/RRT_Display", 10);

  ros::spin();
  return 0;
}
