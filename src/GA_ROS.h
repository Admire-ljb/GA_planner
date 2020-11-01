/**
 * @file GA_ROS.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-10-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <string>

/** include ros libraries**********************/
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
/** ********************************************/ 

#include <boost/foreach.hpp>
/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include "genetic_algorithm/GA.h"
#include <set>
using namespace std;
using std::string;

#ifndef GA_ROS_CPP
#define GA_ROS_CPP

/**
 * @struct cells
 * @brief A struct that represents a cell and its fCost.
 */
struct cells {
  int currentCell;
  float fCost;

};      

namespace GA_planner {
  
class GAPlanner : public nav_core::BaseGlobalPlanner {
public:
  
  GAPlanner(ros::NodeHandle &nh); //this constructor is may be not needed
  GAPlanner();
  GAPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  
  ros::NodeHandle ROSNodeHandle;
  
  /** overriden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal, 
    std::vector<geometry_msgs::PoseStamped>& plan
         );
 

  void getCorrdinate (float& x, float& y);
  int convertToCellIndex (float x, float y);
  void convertToCoordinate(int index, float& x, float& y);
  bool isCellInsideMap(float x, float y);
  void mapToWorld(double mx, double my, double& wx, double& wy);
  vector<int> GAplanner(int startCell, int goalCell);
  vector <int> findFreeNeighborCell (int CellID);
  bool isStartAndGoalCellsValid(int startCell,int goalCell); 
  float getMoveCost(int CellID1, int CellID2);
  float getMoveCost(int i1, int j1, int i2, int j2);
  bool isFree(int CellID); //returns true if the cell is Free
  bool isFree(int i, int j); 

  int getCellIndex(int i,int j) //get the index of the cell to be used in Path
  {
   return (i*width)+j;  
  }
  int getCellRowID(int index)//get the row ID from cell index
  {
     return index/width;
  }
  int getCellColID(int index)//get colunm ID from cell index
  {
    return index%width;
  }
private:
  float originX;
  float originY;
  float resolution;
  costmap_2d::Costmap2DROS* costmap_ros_;
  double step_size_, min_dist_from_robot_;
  costmap_2d::Costmap2D* costmap_;
  bool initialized_;
  int** OGM_data;
  int width;
  int height;
};

};
#endif

