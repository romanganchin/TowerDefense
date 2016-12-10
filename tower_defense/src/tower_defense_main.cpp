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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

/*
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
*/
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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// Publisher for marker messages.
ros::Publisher markers_publisher_;

//Publisher for filtered point cloud
ros::Publisher filtered_point_cloud_publisher_;

// Publisher for 3D point clouds.
ros::Publisher point_cloud_publisher_;
ros::Publisher start_cloud_publisher_;
ros::Publisher end_cloud_publisher_;


// Markers for visualization.
Marker vertices_marker_;
Marker qrand_marker_;
Marker edges_marker_;
Marker map_marker_;
Marker plan_marker_;

//////////////////////////////////////////////////
//              Global Variables                //
//////////////////////////////////////////////////

//Distance of kinect from plane (in meters)
//Real value will be found from pointcloud
//float DISTANCE = 1;
//Holds the location of the plane (Z)
float MODEZ = 0;

//Goal height (maximum height of goal)
float GOAL_H  = .1;

//Wall height (maximum height of walls)
float WALL_H = .13;

//Tower height (maximum height of towers)
float TOWER_H = .25;

//camera intrinsics
float fx = 588.446;
float fy = -564.227;
float px = 320;
float py = 240;
float a = 3.008;
float b = -0.002745;


//////////////////////////////////////////////////
//              DATA STRUCTURES                //
//////////////////////////////////////////////////


//////////////////SUPERPOINTCLOUD/////////////////////////
//Our own pointcloud datastructure with just the
//points and rgb values
struct SUPERPOINTCLOUD
{
  vector<Vector3f> points;
  vector<Vector3f> colors;
};

//The game map which will store information on the entire
//game state

//Unfiltered pointcloud
SUPERPOINTCLOUD POINTCLOUD;

//Filtered pointcloud 
SUPERPOINTCLOUD p; 


///////////////Towers, Start, Goal////////////
struct tower{
  //Middle of the tower (x,y) z is top of tower
  Vector3f location;
  //All the points that comprise the tower
  vector<Vector3f> points;
  //Damage delt to creeps
  int damage;
  //Max range of the tower, how far it can hit
  float range; 
  float height;
};

struct creep{
	Vector3f location;
	float damageTaken;
};
//Constructor method for tower
tower makeTower(Vector3f l, vector<Vector3f> po, int d, float r){
  tower thing;
  thing.location = l;
  thing.points = po;
  thing.damage = d;
  thing.height = MODEZ - l.z();
  return thing;
} 

//For the maping of towers, start and goal:
vector<tower> TOWERS;
Vector3f START(0,0,0);
Vector3f GOAL(0,0,0); 


///////////////////////////////////////////////////
//              Helper Functions                 //
///////////////////////////////////////////////////
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

////////////////////////////////////////////////////
//     Unused Functions (DELETE?)                 //
////////////////////////////////////////////////////

//this does not work! copied the code from previous assignment we need to use pointcloud2's everywhere or transform the pointcloud2 to pointcloud in the dpeth image call back
bool ObstaclePointCloudService(tower_defense::ObstaclePointCloudSrv::Request& req,tower_defense::ObstaclePointCloudSrv::Response& res) {
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


static inline int getPointCloud2FieldIndex (const sensor_msgs::PointCloud2 &cloud, const std::string &field_name)
{
   // Get the index we need
 for (size_t d = 0; d < cloud.fields.size (); ++d){
     if (cloud.fields[d].name == field_name)
      return (d);
  }
  return (-1);
 }


//sensor_msgs/PointCloud2.msg
 /*
void DepthImageCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {
  // Message for published 3D point clouds.
  //this works!!!
  //printf("we are reading the depth image");
  sensor_msgs::PointCloud point_cloud;
  // point_cloud.header = image.header;
  //    for (size_t d = 0; d < image.fields.size (); ++d){
  //       cout << (image.fields[d].name)+ "\n";
  //   }
  //   for (size_t i = 0; i < image.data.size (); ++i)
  //   {
  //     cout << "  image[" << i << "]: ";
  //     cout << "\n  " << image.data[i].x;
  //   }

  // Container for original & filtered data
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(*input,pcl_pc2);
     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    
  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "camera_rgb_optical_frame";
  msg->height = temp_cloud->height;
  msg->width = temp_cloud->width;
  //msg->points.push_back (temp_cloud->points);
    //printf ("Cloud: width = %d, height = %d\n", temp_cloud->width, temp_cloud->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, temp_cloud->points)
      msg->points.push_back (pt);
  // // Convert to PCL data type
  //pcl_conversions::toPCL(*cloud_msg, *cloud);

  // // Perform the actual filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.1, 0.1, 0.1);
  // sor.filter (cloud_filtered);

  // // Convert to ROS data type
  // sensor_msgs::PointCloud2 output;
  // pcl_conversions::moveFromPCL(cloud_filtered, output);

  // // Publish the data
  // point_cloud_publisher_.publish (output);
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
  //     point_cloud.points.push_back(point);
  //   }
  // }
  point_cloud_publisher_.publish(msg);
}
*/

///////////////////////////////////////////////////
//                 Functions                     //
///////////////////////////////////////////////////
void gameMap();
float maxOccuringValue();
/*
	returns the 2d distance between two points, 
	to be used to find the distance between the location of the creep and the location
	of the tower
*/
float distanceBetweenTwoPoints(Vector3f v1, Vector3f v2){
	float diffY = v1.y() - v2.y();
    float diffX = v1.x() - v2.x();
    return sqrt((diffY * diffY) + (diffX * diffX));
}
/*
	Finds the distance of the creep that is closest to the tower
*/
creep getClosestCreep(tower currentTower, vector<creep> creeps){
	float closestDistance = distanceBetweenTwoPoints(currentTower.location, creeps[0].location);
	float currentDistance = closestDistance;
	creep c = creeps[0];
	for(size_t i = 0; i < creeps.size(); i++){
		currentDistance = distanceBetweenTwoPoints(currentTower.location, creeps[i].location);
		if(currentDistance < closestDistance){
			closestDistance = currentDistance;
			c = creeps[i];
		}
	}
	return c;
}
void TowerAI(vector<tower> towers, vector<creep> creeps){
	//variable for message to send to HurtCreepService
	//HurtCreepService takes 
	//int32[] damage
    //geometry_msgs/Point32[] location
    vector<creep> sendToHurtCreeperService;
    creep tempCreep;
	for(size_t i = 0; i < towers.size(); i++){
		tempCreep = getClosestCreep(towers[i], creeps);
		tempCreep.damageTaken = towers[i].damage;
		sendToHurtCreeperService.push_back(tempCreep);
	}
	//sendtoHurtCreeperService here
}
//Determines the mode of the z coordinates for the SUPERPOINTCLOUD
float maxOccuringValue(){
  printf("finding max value\n");
  float currentCount = 0;
  float maxCount = 0;
  float location = 0;
  SUPERPOINTCLOUD pp = POINTCLOUD;
  for(size_t i = 0; i < pp.points.size(); i+=10){
    //printf("Z:%f", p.points[i].z());
    if(isnan(pp.points[i].z()) == 0){
    for(size_t j = 0; j < pp.points.size(); j++){
      if(pp.points[j].z()<=pp.points[i].z()+.1 || pp.points[j].z() >=pp.points[i].z()-.1){
        currentCount++;
      }
    }
    if(currentCount > maxCount){
      maxCount = currentCount;
      location = pp.points[i].z();
    }
    }
    //printf("\nx %f  weight %f\n",_particles[i].x, _particles[i].weight);  
  }
  printf("maxCount ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^%f\n", maxCount);
  printf("location ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^%f\n", location);
  return location;
}
//Takes the PointCloud2 from the Kinect, converts it into PointCloud<pcl::PointXYZRGB>
//Takes this point cloud and store the locations and colors into a SUPERPOINTCLOUD
//Filters the ground plane out of the SUPERPOINTCLOUD
void KinectCallback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >& pointcloud){
  POINTCLOUD.points.clear();
  POINTCLOUD.colors.clear(); 

   BOOST_FOREACH (const pcl::PointXYZRGB& pt, pointcloud->points){
      Vector3f point(pt.x, pt.y, pt.z);
      Vector3f color((float) pt.r, (float)pt.g, (float)pt.b);
      POINTCLOUD.points.push_back(point);
      POINTCLOUD.colors.push_back(color);
     // printf("%f, %d, %d\n", (float)pt.r, pt.g, pt.b);
    }
    //use this ounce to find the z value for the base plane with the intial settup
    if(MODEZ == 0){MODEZ = maxOccuringValue(); printf("Done\n");}

    
    //With the pointcloud found (POINTCLOUD), filter out the base
    //plane and store it in p. 
    p = POINTCLOUD;
    sensor_msgs::PointCloud publish_cloud;
    publish_cloud.header.frame_id = "camera_rgb_optical_frame";
    publish_cloud.points.resize(p.points.size());
    
    sensor_msgs::PointCloud start_cloud;
    start_cloud.header.frame_id = "camera_rgb_optical_frame";
    start_cloud.points.resize(p.points.size());

    sensor_msgs::PointCloud end_cloud;
    end_cloud.header.frame_id = "camera_rgb_optical_frame";
    end_cloud.points.resize(p.points.size());

    for(size_t i = 0; i < p.points.size(); i++){
      //if(p.points[i].z() <= 1.036 || p.points[i].z() >=1.016)
      //z value for recorded bag files 1.026
      //found z value - .01
      if(p.points[i].z() <=1.016)
          publish_cloud.points[i] = ConvertVectorToPoint(p.points[i]);
        if(p.colors[i].x() < 100 && p.colors[i].y() < 100 && p.colors[i].z() > 180){
          start_cloud.points[i] = ConvertVectorToPoint(p.points[i]);
        }
        if(p.colors[i].x() > 180 && p.colors[i].y() < 100 && p.colors[i].z() < 100){
          end_cloud.points[i] = ConvertVectorToPoint(p.points[i]);
        }
    }
    start_cloud_publisher_.publish(start_cloud);
    end_cloud_publisher_.publish(end_cloud);
    point_cloud_publisher_.publish(publish_cloud);
      /*printf("XYZ:%f,%f,%f RGB:%f,%f,%f \n", 
        p.points[i].x(),p.points[i].y(), p.points[i].z(),
        p.colors[i].x(),p.colors[i].y(),p.colors[i].z());*/
    
    //printf("%lu\n", p.colors.size());
    //printf("%lu\n", p.points.size());

    gameMap();
    printf("//////////////\n" );

}

/* NOT DONE YET
void towerFind(SUPERPOINTCLOUD towercloud){
  size_t towMag = towercloud.points.size();
  size_t totalPoints = towMag;
  
  //Only searches for 20 towers
  size_t tower_limit = 20;
  size_t limiter = 0;
  //A tower must have atleast 100 points. 
  size_t towercutoff = 100;
  //Search entire space for 
  while(totalPoints > towercutoff && limiter <= tower_limit){
    limiter++;

    size_t mostPoints = 0;
    tower best_tower; 
    for(int i = 0; i < towMag; i++){
      size_t tempPoints = 0; 
      Vector3f pMid = towercloud.points[i];


      tower temp_tower;
      temp_tower.location = pMid;

      for(int j = 0; j < towMag; j++){
        Vector3f test = towercloud.points[i];
        float distance = sqrt(pow(pMid.x() - test.x(),2) +
                              pow(pMid.y() - test.y(),2) +
                              pow(pMid.z() - test.z(),2));
        if(distance < .02){
          temp_tower.points.push_back(test);
          tempPoints++;
        }
        if(tempPoints > mostPoints){
          best_tower = 
        }
      }
    }

}
}
*/

//Finds the location of the start point, goal point, and towers
//Identifies all towers
void gameMap(){
    SUPERPOINTCLOUD towercloud;
    SUPERPOINTCLOUD startcloud;
    SUPERPOINTCLOUD goalcloud; 

  //Filter points into three point clouds
  //towercloiud for points that make up towers
  //startcloud 


  for(size_t i = 0; i <= p.points.size(); i++){
    Vector3f cur = p.points[i];
    float height = MODEZ - POINTCLOUD.points[i].z();
    if(height > WALL_H){
      towercloud.points.push_back(cur);
    }
    if(height < GOAL_H && height > 0){
      //RGB.x() = red, .y() = Green, .z() = blue
      Vector3f rgb = p.colors[i];
      //If blue, then it is a start
      if(rgb.z() > 120 && rgb.z() > rgb.x()*2 && rgb.z() > rgb.y()*2){
        startcloud.points.push_back(cur);
        startcloud.colors.push_back(rgb);
      }
      //If red then is an end
      if(rgb.x() > 120 && rgb.x() > rgb.z()*1.5 && rgb.x() > rgb.y()*1.5){
        goalcloud.points.push_back(cur);
        goalcloud.colors.push_back(rgb);
      }
    }
  }
  printf("Tower: %lu\n", towercloud.points.size());
  printf("Start: %lu\n", startcloud.points.size());
  printf("Goal: %lu\n", goalcloud.points.size());

  //To find start and goal, find the average of all the points
  //in their respective arrays

  size_t startMag = startcloud.points.size();
  Vector3f startPoint(0,0,0);
  for(size_t i = 0; i < startMag; i++){
    Vector3f cur = startcloud.points[i];
    startPoint.x() += cur.x();
    startPoint.y() += cur.y();
    startPoint.z() += cur.z();
  }
  startPoint.x() = startPoint.x()/(float) startMag;
  startPoint.y() = startPoint.y()/(float) startMag;
  startPoint.z() = startPoint.z()/(float) startMag;


  size_t goalMag = goalcloud.points.size();
  Vector3f goalPoint(0,0,0);
  for(size_t i = 0; i < goalMag; i++){
    Vector3f cur = goalcloud.points[i];
    goalPoint.x() += cur.x();
    goalPoint.y() += cur.y();
    goalPoint.z() += cur.z();
  }
  goalPoint.x() = goalPoint.x()/(float) goalMag;
  goalPoint.y() = goalPoint.y()/(float) goalMag;
  goalPoint.z() = goalPoint.z()/(float) goalMag;

  printf("Start: %f,%f,%f\n", startPoint.x(), startPoint.y(), startPoint.z());
  printf("Goal: %f,%f,%f\n", goalPoint.x(), goalPoint.y(), goalPoint.z());
}


//////////////////////////////////////////////////
//                    main                      //
//////////////////////////////////////////////////

int main(int argc, char **argv) {
  InitMarkers();

  ros::init(argc, argv, "camera_rgb_optical_frame");
  ros::NodeHandle n;

  filtered_point_cloud_publisher_ = 
    n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/FilteredPointCloud", 1);

  point_cloud_publisher_ =
    n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/PointCloud", 1);
 
 start_cloud_publisher_ =
  n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/StartCloud", 1);

end_cloud_publisher_ =
  n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/EndCloud", 1);

  //this works!
  ros::Subscriber depth_image_subscriber =
    n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, KinectCallback);

  // markers_publisher_ = n.advertise<visualization_msgs::MarkerArray>(
  //     "/COMPSCI403/RRT_Display", 10);
   // gameMap();

  ros::spin();
  return 0;
}
