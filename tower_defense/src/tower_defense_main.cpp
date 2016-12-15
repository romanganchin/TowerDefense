#include <algorithm>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
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
#include <std_msgs/Float32.h>

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
#include "tower_defense/HurtCreeperSrv.h"
#include "tower_defense/MoveCreepersSrv.h"
#include "tower_defense/MakePathSrv.h"
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
using geometry_msgs::Point32;
using geometry_msgs::Point;
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
ros::Publisher make_path_publisher;
ros::Publisher my_path_publisher_;
ros::Publisher my_points_publisher_;
ros::Publisher creep_publisher_;
//Service callers
ros::ServiceClient hurt_creeper;
ros::ServiceClient move_creeper;
ros::ServiceClient get_path_caller;
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

bool MAKEPATH = true;

//Goal height (maximum height of goal)
float GOAL_H  = .1;

//Wall height (maximum height of walls)
float WALL_H = .14;

//Tower height (maximum height of towers)
float TOWER_H = .25;

//camera intrinsics
float fx = 588.446;
float fy = -564.227;
float px = 320;
float py = 240;
float a = 3.008;
float b = -0.002745;
bool _make_path_check = true;

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
  size_t location;
  float health;
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

struct edge{
    Vector3f vertexA;
    Vector3f vertexB;
};

//For the maping of towers, start and goal:
vector<tower> TOWERS;
Vector3f START(0,0,0);
Vector3f GOAL(0,0,0); 
vector<Vector3f> PATH;
vector<creep> HORDE;
std::vector<Vector3f> POSSIBLE;

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
Point VectorToPoint(const Vector3f& v) {
  Point point;
  point.x = v.x();
  point.y = v.y();
  point.z = v.z();
  return point;
}

// Helper function to visualize a point.
void DrawPoint(const Vector3f& p, Marker* marker) {
  marker->points.push_back(VectorToPoint(p));
}

// Helper function to visualize an edge.
void DrawLine(const Vector3f& p1,
              const Vector3f& p2,
              Marker* marker) {
  marker->points.push_back(VectorToPoint(p1));
  marker->points.push_back(VectorToPoint(p2));
}

// Initialize all markers.
void InitMarkers() {
  vertices_marker_.header.frame_id = "camera_rgb_optical_frame";
  vertices_marker_.id = 1;
  vertices_marker_.type = Marker::POINTS;
  vertices_marker_.action = Marker::MODIFY;
  vertices_marker_.scale.x = 0.01;
  vertices_marker_.scale.y = 0.01;
  vertices_marker_.color.a = 1.0;
  vertices_marker_.color.r = 0.0;
  vertices_marker_.color.g = 0.0;
  vertices_marker_.color.b = 1.0;


  qrand_marker_.header.frame_id = "camera_rgb_optical_frame";
  qrand_marker_.id = 2;
  qrand_marker_.type = Marker::POINTS;
  qrand_marker_.action = Marker::MODIFY;
  qrand_marker_.scale.x = 0.005;
  qrand_marker_.scale.y = 0.005;
  qrand_marker_.color.a = 1.0;
  qrand_marker_.color.r = 1.0;
  qrand_marker_.color.g = 0.0;
  qrand_marker_.color.b = 0.0;

  edges_marker_.header.frame_id = "camera_rgb_optical_frame";
  edges_marker_.id = 3;
  edges_marker_.type = Marker::LINE_LIST;
  edges_marker_.action = Marker::MODIFY;
  edges_marker_.scale.x = 0.01;
  edges_marker_.scale.y = 0.01;
  edges_marker_.color.a = 1.0;
  edges_marker_.color.r = 0.0;
  edges_marker_.color.g = 1.0;
  edges_marker_.color.b = 0.0;

  map_marker_.header.frame_id = "camera_rgb_optical_frame";
  map_marker_.id = 4;
  map_marker_.type = Marker::LINE_LIST;
  map_marker_.action = Marker::MODIFY;
  map_marker_.scale.x = 0.05;
  map_marker_.scale.y = 0.05;
  map_marker_.color.a = 1.0;
  map_marker_.color.r = 1.0;
  map_marker_.color.g = 1.0;
  map_marker_.color.b = 1.0;

  plan_marker_.header.frame_id = "camera_rgb_optical_frame";
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


///////////////////////////////////////////////////
//                 Functions                     //
///////////////////////////////////////////////////
//moveCreep returns true if a creep reaches the end
bool moveCreep(bool make_new){
  size_t pathsize = PATH.size();
  size_t hordesize = HORDE.size();
  bool gameOver = false;
  if(make_new){
    creep new_creep;
    new_creep.location = pathsize -1;
    new_creep.health = 10;
    HORDE.push_back(new_creep);
  }

  for(size_t i = 0; i < hordesize; i++){
    HORDE[i].location--;
    if(HORDE[i].location <= 0){
      gameOver = true; 
    }
  }
  return gameOver;
}

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
edge hurtClosestCreep(tower currentTower){
  vector<creep> creeps = HORDE;
  edge edge;
  Vector3f temp(-9, -9, -9);
  if(HORDE.size()>0){
	  float closestDistance = distanceBetweenTwoPoints(currentTower.location, PATH[creeps[0].location]);
	  float currentDistance = closestDistance;

	  edge.vertexA = temp;
	  edge.vertexB = temp;
	  size_t creep_in = 0;
	  for(size_t i = 0; i < creeps.size(); i++){
	    currentDistance = distanceBetweenTwoPoints(currentTower.location, PATH[creeps[i].location]);
	    if(currentDistance < closestDistance){
	      closestDistance = currentDistance;
	      creep_in = i;
	    }
	  }
	  if(closestDistance < currentTower.range){
	    HORDE[creep_in].health -= currentTower.damage;
	    edge.vertexA = currentTower.location;
	    edge.vertexB = PATH[HORDE[creep_in].location];

	    if(HORDE[creep_in].health <= 0){
	      // vector<creep> temp;
	      // for(size_t i = 0; i < HORDE.size(); i++){
	      //   if(i != creep_in){
	      //     temp.push_back(HORDE[i]);
	      //   }
	      // }
	      // HORDE = temp;
	      HORDE.erase(HORDE.begin() + creep_in);
	    }
	  }
	}
  return edge;
}
/*
* This is working gets all the points within a certain radius
*/
vector<Vector3f> GetPointsToConsider(Vector3f current, vector<Vector3f> possibleLocations,float distance){
	vector<Vector3f> pointsToConsider;
	for(size_t i = 0; i<possibleLocations.size(); i++){
		if(distanceBetweenTwoPoints(current, possibleLocations[i])<=distance){
			pointsToConsider.push_back(possibleLocations[i]);
		}
	}
	//printf("got to the points to consider\n");
	return pointsToConsider;
}
Vector3f GetPointClosestToGoal(vector<Vector3f> pointsToConsider, Vector3f myGoal, bool goaltostart){
	//print points to Consider should be radius + already seen points
	sensor_msgs::PointCloud my_points_cloud;
    my_points_cloud.header.frame_id = "camera_rgb_optical_frame";
    my_points_cloud.points.resize(pointsToConsider.size());
    for(size_t i = 0; i < pointsToConsider.size(); i++){
   		my_points_cloud.points[i] = ConvertVectorToPoint(pointsToConsider[i]);
    }
    my_points_publisher_.publish(my_points_cloud);

      float random = RandomValue(0,1);
      Vector3f g;

    float chance = .7;
    if(goaltostart){
      chance = .98;
    }

	  if(random < chance){
	  	g = myGoal;
	  }
	  else{
	  	g.x() = RandomValue(-0.5, .5);
	  	g.y() = RandomValue(-.5, .5);
	  	g.z() = MODEZ;
	  }
	  float closestDistance = distanceBetweenTwoPoints(g, pointsToConsider[0]);
	  float currentDistance = closestDistance;
	  Vector3f c(0,0,MODEZ);
	  //printf("points to consider Size%lu \n", pointsToConsider.size());
	  for(size_t i = 0; i < pointsToConsider.size(); i++){
	  	// if(pointsToConsider[i] == GOAL){
	  	// 	c.pointsToConsider[i];
	  	// }
	    currentDistance = distanceBetweenTwoPoints(g, pointsToConsider[i]);

	    if(currentDistance <= closestDistance){
	      closestDistance = currentDistance;
	      c = pointsToConsider[i];
	    }
	  }
  	return c;
}

vector<Vector3f> MakePath(vector<Vector3f> possibleLocations, Vector3f s, Vector3f g, bool goaltostart){
	
	printf("making path \n");
	vector<Vector3f> pointsOnPath;
	Vector3f current = s;
	printf("disance between goal and start %f \n", distanceBetweenTwoPoints(g, current));
	pointsOnPath.push_back(current);
	vector<Vector3f> pointsToConsider;
	bool not_reached_goal = true;
	int counter = 0;
	// vector<Vector3f> radiusPoints;
	// float radiusDistance = .01;
	float distance = .025;
	// pointsToConsider = GetPointsToConsider(current, possibleLocations, distance);
	// printf("radius Size %lu \n", pointsToConsider.size());
	// int radiusSize = 550;//pointsToConsider.size();
	// int currentRadiusSize = 0;
	while(not_reached_goal  && counter < 10000){
		//get all points within a certain radius of the current location that are valid points
		int radiusCounter = 0;
		pointsToConsider = GetPointsToConsider(current, possibleLocations, distance);
		for(size_t i = 0; i < pointsOnPath.size(); i++){
			pointsToConsider.push_back(pointsOnPath[i]);
		}
		//currentRadiusSize = pointsToConsider.size();
		current = GetPointClosestToGoal(pointsToConsider, g, goaltostart);
		/*
		Uncomment for radius checking
		*/
		// while(fabs(radiusSize-currentRadiusSize)>50 && radiusCounter < 50){
		// 	radiusPoints = GetPointsToConsider(current, possibleLocations, distance);
			
		// 	currentRadiusSize = radiusPoints.size();
			
		// 	current = GetPointClosestToGoal(pointsToConsider, g);
		// 	radiusCounter++;
		// // }
		// printf("current radius Size %lu \n", currentRadiusSize);
		// printf("original Radius Size %i \n", radiusSize);
		//radiusPoints = GetPointsToConsider(current, )
		//DrawLine(current.location, pointsToConsider[current.parent], &edges_marker_);
		pointsOnPath.push_back(current); 
		if(.01 > distanceBetweenTwoPoints(g, current)){
            not_reached_goal = false;
        }
        counter++;
	}
	printf("disance between goal and start %f \n", distanceBetweenTwoPoints(g, current));
	printf("got to the publisher\n");
	printf("got to the publisher\n");
		
    sensor_msgs::PointCloud my_path_cloud;
    my_path_cloud.header.frame_id = "camera_rgb_optical_frame";
    my_path_cloud.points.resize(pointsOnPath.size());
    for(size_t i = 0; i < pointsOnPath.size(); i++){
   		my_path_cloud.points[i] = ConvertVectorToPoint(pointsOnPath[i]);
    }
    my_path_publisher_.publish(my_path_cloud);
	return pointsOnPath;
}
bool TowerAI(){
    bool victory = false;

    MarkerArray markers;
     markers.markers.clear();
     plan_marker_ = edges_marker_;
    vector<tower> towers = TOWERS;
    creep tempCreep;
    for(size_t i = 0; i < towers.size(); i++){
      edge line = hurtClosestCreep(towers[i]);
      //printf("%f, %f\n", line.vertexB.x(), line.vertexB.y());
      if(line.vertexA.x() != -9){
        DrawLine(line.vertexA, line.vertexB, &plan_marker_);
        markers.markers.push_back(plan_marker_);
      }
    //tempCreep = getClosestCreep(towers[i]);
    //tempCreep.damageTaken = towers[i].damage;
    //sendToHurtCreeperService.push_back(tempCreep);
  }
  if(HORDE.size() <= 0){
    victory = true;
  }

  
    markers_publisher_.publish(markers);

  return victory;
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
    for(size_t j = 0; j < pp.points.size(); j+= 5){
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
   	  if(isnan(pt.x) == 0 && isnan(pt.y) == 0 && isnan(pt.z) == 0){
	      Vector3f point(pt.x, pt.y, pt.z);
	      Vector3f color((float) pt.r, (float)pt.g, (float)pt.b);
	      POINTCLOUD.points.push_back(point);
	      POINTCLOUD.colors.push_back(color);
  		}
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

    sensor_msgs::PointCloud ground_cloud;
    ground_cloud.header.frame_id = "camera_rgb_optical_frame";
    ground_cloud.points.resize(p.points.size());

    //sendto make path service here
  	tower_defense::MakePathSrv srv;
  	

  	vector<Vector3f> possibleLocations;
  	vector<Vector3f> pointsSendingForRequest;
    for(size_t i = 0; i < p.points.size(); i++){
      //if(p.points[i].z() <= 1.036 || p.points[i].z() >=1.016)
      //z value for recorded bag files 1.026
      //found z value - .01
		if(isnan(p.points[i].x()) == 0 && isnan(p.points[i].y()) == 0 && isnan(p.points[i].z()) == 0){
			if(p.points[i].z() > MODEZ-.05 && p.points[i].z() < MODEZ+.05){
				//ground_cloud.points[i] = ConvertVectorToPoint(p.points[i]);
				possibleLocations.push_back(p.points[i]);
			//srv.request.point_cloud[i] = ConvertVectorToPoint(p.points[i]);
			}
			if(p.points[i].z() <=MODEZ-.05){
				publish_cloud.points[i] = ConvertVectorToPoint(p.points[i]);
				pointsSendingForRequest.push_back(p.points[i]);
				//srv.request.point_cloud[i] = ConvertVectorToPoint(p.points[i]);
			}
			if(p.colors[i].x() < 100 && p.colors[i].y() < 100 && p.colors[i].z() > 180){
			start_cloud.points[i] = ConvertVectorToPoint(p.points[i]);
			}
			if(p.colors[i].x() > 180 && p.colors[i].y() < 100 && p.colors[i].z() < 100){
				end_cloud.points[i] = ConvertVectorToPoint(p.points[i]);
			}
		}
    }
    srv.request.point_cloud.resize(pointsSendingForRequest.size());
    for(size_t i = 0; i< pointsSendingForRequest.size();i++){
    	srv.request.point_cloud[i] = ConvertVectorToPoint(pointsSendingForRequest[i]);
    }

    if(_make_path_check){
	   	//_make_path_check = false;
	    gameMap();
	    //end_cloud.points[0] = ConvertVectorToPoint(GOAL);
    	
	    possibleLocations.push_back(START);
	    possibleLocations.push_back(GOAL);
      POSSIBLE = possibleLocations;

	    Vector3f s = START;
	    Vector3f g = GOAL;
	   
      start_cloud_publisher_.publish(start_cloud);
    end_cloud_publisher_.publish(end_cloud);
	}
	ground_cloud.points.resize(PATH.size());
	for(size_t i = 0; i < PATH.size(); i++){
		ground_cloud.points[i] = ConvertVectorToPoint(PATH[i]);
	}
    // ground_cloud.points[p.points.size()-2] = (ConvertVectorToPoint(START));
    // ground_cloud.points[p.points.size()-1] = (ConvertVectorToPoint(GOAL));
    filtered_point_cloud_publisher_.publish(ground_cloud);
    
    //end_cloud_publisher_.publish();
    point_cloud_publisher_.publish(publish_cloud);
      /*printf("XYZ:%f,%f,%f RGB:%f,%f,%f \n", 
        p.points[i].x(),p.points[i].y(), p.points[i].z(),
        p.colors[i].x(),p.colors[i].y(),p.colors[i].z());*/
    
    //printf("%lu\n", p.colors.size());
    //printf("%lu\n", p.points.size());

    
   if(_make_path_check){
   	_make_path_check = false;

  //resize
  srv.request.start = ConvertVectorToPoint(START);
  srv.request.end = ConvertVectorToPoint(GOAL);
  //srv.request.point_cloud = ground_cloud;
  float x_max = 0;
  float x_min = 1000;
  float y_max = 0;
  float y_min = 1000;
  float temp;
  //convert to srv type HurtCreeperSrv
  for(size_t i = 0; i < p.points.size(); i++){
   //srv.request.point_cloud[i] = ConvertVectorToPoint(p.points[i]);
   if(isnan(p.points[i].x()) == 0 && isnan(p.points[i].y()) == 0){
	   temp = p.points[i].x();
	   if(temp < x_min){
	   	x_min = temp;
	   }
	   if(temp > x_max){
	   	x_max = temp;
	   }
	   temp = p.points[i].y();
	   if(temp < y_min){
	   	y_min = temp;
	   }
	   if(temp > y_max){
	   	y_max = temp;
	   }
	}
  }
  printf("borders: xMin %f xMax %f yMin %f yMax %f\n",x_min, x_max, y_min, y_max);
  /*
  if (get_path_caller.call(srv))
  {
   //not sure if you want the result from the service somewhere else
   //the service returns creeper locations
   //if you want the result somewhere else either make it a global or 
   //just have towerai method return sendtoHurtCreeperService
  	sensor_msgs::PointCloud make_path;
    make_path.header.frame_id = "camera_rgb_optical_frame";
  	make_path.points.resize(srv.response.path.size());
  	for(size_t i = 0; i < srv.response.path.size(); i++){
  		make_path.points[i] = srv.response.path[i];
  		make_path.points[i].z = MODEZ;
  		printf("x %f y %f z %f\n",make_path.points[i].x, make_path.points[i].y, make_path.points[i].z);
  	}
  	make_path_publisher.publish(make_path);
  }
  else
  {
   ROS_ERROR("Failed to call service make path service");
  } 
}
    printf("//////////////\n" );
    */
  }
}

//Given: a pointcloud with the points which comprise all the 
//towers in the game space 
//Identifies each tower in the space and pushing to the TOWERS vector
void towerFind(SUPERPOINTCLOUD tcloud, size_t towercutoff, float radius){
  printf("TOWER FIND\n");
  size_t towMag = tcloud.points.size();
  size_t totalPoints = towMag;

  if(towMag < towercutoff){printf("ERROR TOWERFIND: NOT ENOUGH POINTS\n"); return;}
  SUPERPOINTCLOUD preCloud = tcloud;
 
  vector<tower> newTowers;
  TOWERS = newTowers;
  
  //Only searches for 10 towers
  size_t tower_limit = 10;
  size_t limiter = 0;

  if(TOWERS.size() > 20000){
    printf("To many towers!\n");
    return;
  }
   
  //Tracks best tower information
  size_t mostPoints = 1;
  vector<Vector3f> bestcolors;
  bestcolors = tcloud.colors;
  tower best_tower;
  //Search entire space for
  while(totalPoints > towercutoff && limiter <= tower_limit){
    //Increment limiter
    limiter++;

    //preCloud saves the state of flagged and unflagged points
    //A flagged point is already included in a tower, and cannot be 
    //added to another tower. 
    if(mostPoints == 0){printf("NO MORE TOWERS\n");break; }
    if(mostPoints > towercutoff){
      preCloud.colors = bestcolors;
      TOWERS.push_back(best_tower);
      printf("Got One %lu\n", mostPoints);
     // printf("LOCA %f, %f, %f\n", best_tower.location.x(), 
      //  best_tower.location.y(), best_tower.location.z());
      
    }
    
    
    //Tracks the tower of bestfit
    mostPoints = 0;
    

    //For every point in towercloud if it is a a good center location
    //for a tower of best fit
    for(size_t i = 0; i < towMag; i++){
      SUPERPOINTCLOUD towercloud = preCloud;
      //Tracks potential towers of best fit
      size_t tempPoints = 0; 

      //The point to be tested
      Vector3f pMid = towercloud.points[i];

      //Store information on potential towers
      tower temp_tower;
      temp_tower.location = pMid;
      if(towercloud.colors[i].x() == 0){
        //Check to see how many points are included in the tower 
        //(with in) a radius around pMid
        for(size_t j = 0; j < towMag; j++){
          //Checks the flag on the current point
          if (towercloud.colors[j].x() == 0){
            //The point to be included or unincluded
            Vector3f test = towercloud.points[j];
            float distance = sqrt(pow(pMid.x() - test.x(),2) +
                                  pow(pMid.y() - test.y(),2) +
                                  pow(pMid.z() - test.z(),2));
            
            if(distance < radius){
              //If with the radius, push the point to the temp
              //towers array of points
              temp_tower.points.push_back(test);
              //Increase the count of the number of points
              //In the current tower
              tempPoints++;
              //Flag as included
              Vector3f newF(1,0,0);
              
              //CHANGE TOWER STATS
              temp_tower.damage = 5;
              temp_tower.range = 1;
              temp_tower.height = fabs(MODEZ - pMid.z());

              towercloud.colors[j] = newF;
            }
            //If the temp tower is the tower of bestfit for pmid 
            //Set it as the tower of best fit
            if(tempPoints > mostPoints && tempPoints > towercutoff){
              mostPoints = tempPoints;
              best_tower = temp_tower;
              bestcolors = towercloud.colors;
            }
          }
        }
      }
    }
  }
  printf("TOWER DONE\n");
}



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
      float height = MODEZ - cur.z();
    
      if(height > WALL_H && cur.z() < MODEZ){
        towercloud.points.push_back(cur);
        //We are going to use the color red as a flag in find tower
        Vector3f color(0,0,0);
        towercloud.colors.push_back(color);
      }
      if(height < GOAL_H && height > 0.001){
        //RGB.x() = red, .y() = Green, .z() = blue
        Vector3f rgb = p.colors[i];

        //printf("%f %f %f\n", rgb.x(), rgb.y(), rgb.z());
        //If blue, then it is a start
        if(rgb.z() > 180 && 100 > rgb.x() && 100 > rgb.y()){
         
          startcloud.points.push_back(cur);
          startcloud.colors.push_back(rgb);
        }
        //If red then is an end
        if(rgb.x() > 180 && 100 > rgb.z() && 100 > rgb.y()){
         
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
      
      //printf("S %f, %f, %f \n", startPoint.x(), startPoint.y(), startPoint.z());
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
      //printf("G %f, %f, %f \n", goalPoint.x(), goalPoint.y(), goalPoint.z());
      goalPoint.x() += cur.x();
      goalPoint.y() += cur.y();
      goalPoint.z() += cur.z();
    }
    goalPoint.x() = goalPoint.x()/(float) goalMag;
    goalPoint.y() = goalPoint.y()/(float) goalMag;
    goalPoint.z() = goalPoint.z()/(float) goalMag;

    printf("Start: %f,%f,%f\n", startPoint.x(), startPoint.y(), startPoint.z());
    printf("Goal: %f,%f,%f\n", goalPoint.x(), goalPoint.y(), goalPoint.z());
    START = startPoint;
    GOAL = goalPoint;

    //Pass the tower cloud and the minimum number of points to make up a tower
    towerFind(towercloud, 100, .05);
    printf("Towers %lu\n", TOWERS.size());

}



void PlayGameCallBack(const std_msgs::Float32& total_creeps){
  bool lost = false;
  bool won = false;

  size_t creeps_made = (size_t) total_creeps.data;
  size_t creeps_left = creeps_made;
  //tower_defense::MoveCreepersSrv moveCreeps; 

  vector<creep> delet;
  HORDE = delet;

  bool makeNew = false;

  size_t frames_per_creep = 1; 
  size_t frame = 0;
  ros::Rate loop_rate(5);

  vector<Vector3f> bestPath;
  size_t minpath = 500;

  size_t counter = 20;
  while(MAKEPATH && counter > 0){
    Vector3f s = START;
    Vector3f g = GOAL;
    vector<Vector3f> poss;
    PATH = poss;
    poss = POSSIBLE;
    vector<Vector3f> pathTree = MakePath(poss, s, g, false);
    //pathTree.push_back(g);
    PATH = MakePath(pathTree, g, s, true);
    if(PATH.size() < minpath){
      bestPath = PATH;
      minpath = PATH.size();
      PATH = bestPath;
      if(PATH.size() < 50){
        MAKEPATH = false;
        /*
        tower trump_tower; 
        Vector3f g(-.023,.3,1);
        trump_tower.location = g;
        trump_tower.damage = 1;
        trump_tower.range = .2;
        TOWERS.push_back(trump_tower); 
        tower newTower; 
        Vector3f h(-.25,.2,1);
        newTower = trump_tower;
        newTower.location = h;
        newTower.damage = 3;
        newTower.range = 1;
        TOWERS.push_back(newTower); 
        */
    }
    }
  }

  
  
  
  while(!lost && !won){
    
    //Spawn 1 creep for every frame_per_creep number of frames 
    if(creeps_made != 0){
      makeNew = true;
      creeps_made--;
    }
    else{
      //If not spawning creeps, move the creeps
     makeNew = false;
    }
    frame++;

    //Call Move Creep Service 
    lost = moveCreep(makeNew);
    if(lost){break;}

    won = TowerAI();
      
    if(won && creeps_made <= 0){
      won = true;
    }
    else(won = false);
    printf("%lu\n", PATH.size());

    //Draw to rviz
    //drawScene(undeadScourge);

    //Convert UndeadScourge to a pointcloud
    //TODO ADD 2 PUBLISHERS
    //creep_publisher which publishes a pointcloud
    //goal_publisher publishes marker
    //ros::Publisher marker_pub = n.advertise<visualization_msg::Marker>("visualization_marker", 100)

    sensor_msgs::PointCloud riseMyMinions;
    riseMyMinions.points.clear();
      riseMyMinions.header.frame_id = "camera_rgb_optical_frame";
      riseMyMinions.points.resize(HORDE.size());
      for(size_t i = 0; i < HORDE.size(); i++){
        riseMyMinions.points[i] = ConvertVectorToPoint(PATH[HORDE[i].location]);
        riseMyMinions.points[i].z = MODEZ -.01;
      }
    creep_publisher_.publish(riseMyMinions);

    for(size_t i = 0; i < HORDE.size(); i++){
      printf("L %lu, H %f\n", HORDE[i].location, HORDE[i].health);
    }

    loop_rate.sleep();
  }

  if(lost){
    printf("DEFEAT: THE UNDEAD HORDE HAS LAID WASTE TO YOUR CITY\n AND ATE ALL YOUR CHEESES! (EVEN THE GOUDA!) \n");
  }
  else{
    printf("VICTORY: NOW TO FEED ON THE FLESH OF YOUR FALLEN FOES\n FRET NOT, WE HAVE CHEESE! (GOUDA FOR ALL!)\n");
  }
}



//////////////////////////////////////////////////
//                  Testing                     //
/////////////////////////////////////////////////

void testingSuite(){
  //////////////////test_simple///////////////
  SUPERPOINTCLOUD test_simple;
  for(int i = 0; i < 100; i++){
    Vector3f point(0,0,0);
    test_simple.points.push_back(point);
    test_simple.colors.push_back(point);
  }
  int counter = 0;
  for(int j = 0; j < 10; j++){
    for(int i = 0; i < 10; i++){
      test_simple.points[counter].x() = i;
      test_simple.points[counter].y() = j;
      counter++;
    }
  }
  //////////////////test_noise///////////////
  SUPERPOINTCLOUD test_noise;
  test_noise = test_simple;
  for(int i = 0; i < 100; i++){
    if(i % 7 == 1){test_noise.points[i].z() = -.05;}
  }

  //////////////////test_mini////////////////
  SUPERPOINTCLOUD test_mini;
  test_mini = test_simple;
  counter = 0;
  
  for(int j = 0; j < 10; j++){
    for(int i = 0; i < 10; i++){
      //Make Walls
      if(counter%10 == 0 || counter%10 == 9 || j == 0 || j == 9){
        test_mini.points[counter].z() =-.10;
      }
      if(counter == 11|| counter == 12 || counter == 21|| counter ==22){
        test_mini.points[counter].z() = -.05;
        test_mini.colors[counter].z() = 200;
        test_mini.colors[counter].y() = 10;
        test_mini.colors[counter].x() = 10;
      }
      if(counter == 54|| counter == 55 || counter ==64 || counter == 65){
        test_mini.points[counter].x() = test_mini.points[counter].x()*.01;
        test_mini.points[counter].y() = test_mini.points[counter].y()*.01;
        test_mini.points[counter].z() = -.20;
        
      }
      if(counter == 77|| counter == 78 || counter == 87|| counter == 88){
        test_mini.points[counter].z() = -.05;
        test_mini.colors[counter].x() = 200;
        test_mini.colors[counter].y() = 10;
        test_mini.colors[counter].z() = 10;
      }
      //printf("%f", test_mini.points[counter].z());
      counter++;
    }
    //printf("\n");
  }
  ////////////////////////test_twotowers/////////////////
  
  SUPERPOINTCLOUD test_twotowers = test_mini;
  test_twotowers.points[0].z() = -.20;
  test_twotowers.points[0].x() *= .01;
  test_twotowers.points[0].y() *= .01;

  test_twotowers.points[1].z() = -.20;
  test_twotowers.points[1].x() *= .01;
  test_twotowers.points[1].y() *= .01;

  test_twotowers.points[10].z() = -.20;
  test_twotowers.points[10].x() *= .01;
  test_twotowers.points[10].y() *= .01;

  test_twotowers.points[11].z() = -.20;
  test_twotowers.points[11].x() *= .01;
  test_twotowers.points[11].y() *= .01;

  ////////TESTS FOR maxoccurringValue///////
  POINTCLOUD = test_simple;
  MODEZ = maxOccuringValue();
  printf("maxOccuringValue Tests\n");
  printf("Test 1: Expected 0: Found = %f", MODEZ);
  if(MODEZ == 0){printf(" PASSED!\n");}
  else printf("FAILED\n");

  POINTCLOUD = test_noise;
  MODEZ = maxOccuringValue();
  printf("Test 2: Expected 0: Found = %f", MODEZ);
  if(MODEZ == 0){printf(" PASSED!\n");}
  else printf("FAILED\n");

  //////////TESTS FOR gameMap and Towerfind///////////////
  p = test_mini;
  gameMap();
  printf("gameMap and Towerfind Tests\n");
  printf("Test 1: Start(1.5, 1.5, -.05): Found = %f, %f, %f\n", START.x(), START.y(), START.z());
  printf("Test 2: GOAL(7.5, 7.5, -.05): Found = %f, %f, %f\n", GOAL.x(), GOAL.y(), GOAL.z());
  SUPERPOINTCLOUD towercloud;
  towercloud.points.push_back(p.points[54]);
  towercloud.points.push_back(p.points[55]);
  towercloud.points.push_back(p.points[64]);
  towercloud.points.push_back(p.points[65]);
  towercloud.colors.push_back(p.colors[54]);
  towercloud.colors.push_back(p.colors[54]);
  towercloud.colors.push_back(p.colors[54]);
  towercloud.colors.push_back(p.colors[54]);
  towerFind(towercloud, 2, .05);
  printf("Test 3: 1 Tower: Tower = %lu\n", TOWERS.size());
  printf("Test 4: Loc(.04, .05, -.20) Location(%f, %f, %f ) \n", TOWERS[0].location.x(), 
    TOWERS[0].location.y(), TOWERS[0].location.z()) ;

  p = test_twotowers;
  gameMap();
  printf("gameMap Tests\n");
  SUPERPOINTCLOUD towercloud2;
  Vector3f color(0,0,0);
  towercloud2.points.push_back(p.points[54]);
  towercloud2.points.push_back(p.points[55]);
  towercloud2.points.push_back(p.points[64]);
  towercloud2.points.push_back(p.points[65]);
    towercloud2.points.push_back(p.points[0]);
  towercloud2.points.push_back(p.points[1]);
  towercloud2.points.push_back(p.points[10]);
  towercloud2.points.push_back(p.points[11]);
  towercloud2.colors.push_back(color);
  towercloud2.colors.push_back(color);
  towercloud2.colors.push_back(color);
  towercloud2.colors.push_back(color);
  towercloud2.colors.push_back(color);
  towercloud2.colors.push_back(color);
  towercloud2.colors.push_back(color);
  towercloud2.colors.push_back(color);
  towerFind(towercloud2, 2, .05);
  printf("Test 5: 2 Tower: Tower = %lu\n", TOWERS.size());
  printf("Test 6: Loc(.04, .05, -.20) Location(%f, %f, %f ) \n", TOWERS[0].location.x(), 
    TOWERS[0].location.y(), TOWERS[0].location.z()) ;
  printf("Test 7: Loc (0,0,-.20) Location(%f, %f, %f ) \n", TOWERS[1].location.x(), 
    TOWERS[1].location.y(), TOWERS[1].location.z()) ;
  
}


//////////////////////////////////////////////////
//                    main                      //
//////////////////////////////////////////////////

int main(int argc, char **argv) {
  InitMarkers();
  if(argc > 1){
    printf("TESTING\n");
    testingSuite();
    return 0;
  }
  //Seed Random number generator
  srand(time(NULL));
  printf("Starting\n");
  ros::init(argc, argv, "camera_rgb_optical_frame");
  ros::NodeHandle n;

   markers_publisher_ = n.advertise<visualization_msgs::MarkerArray>(
  "/COMPSCI403/RRT_Display", 10);

  filtered_point_cloud_publisher_ = 
    n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/FilteredPointCloud", 1);

  point_cloud_publisher_ =
    n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/PointCloud", 1);
 
 start_cloud_publisher_ =
  n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/StartCloud", 1);

end_cloud_publisher_ =
  n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/EndCloud", 1);

make_path_publisher =
  	n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/MakePath", 1);

  my_path_publisher_ = 
  	n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/MyPath", 1);
  	my_points_publisher_ =
  	n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/MyPoints", 1);


  creep_publisher_ = n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/Creep", 1);

  //this works!
  ros::Subscriber depth_image_subscriber =
    n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, KinectCallback);

  hurt_creeper =
  n.serviceClient<tower_defense::HurtCreeperSrv>("/tower_defense/HurtCreeper");

  move_creeper = 
  n.serviceClient<tower_defense::MoveCreepersSrv>("/tower_defense/MoveCreeper");

  get_path_caller =
  n.serviceClient<tower_defense::MakePathSrv>("/tower_defense/MakePath");
  // markers_publisher_ = n.advertise<visualization_msgs::MarkerArray>(
  //     "/COMPSCI403/RRT_Display", 10);
   // gameMap();

  ros::Subscriber play_game_sub =
   n.subscribe("/tower_defense/PlayGame", 1, PlayGameCallBack);


  ros::spin();
  return 0;
}
