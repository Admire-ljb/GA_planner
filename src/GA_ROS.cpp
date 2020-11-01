
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include "GA_ROS.h"

#include <pluginlib/class_list_macros.h>
//register this planner as a BaseGlobalPlanner plugin


namespace GA_planner
{
int value;
int mapSize;
bool* OGM;
static const float INFINIT_COST = INT_MAX; 
float infinity = std::numeric_limits< float >::infinity();
float tBreak;  

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec-start.tv_nsec)<0) {
    temp.tv_sec = end.tv_sec-start.tv_sec-1;
    temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec-start.tv_sec;
    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
  }
  return temp;
}

PLUGINLIB_EXPORT_CLASS(GA_planner::GAPlanner, nav_core::BaseGlobalPlanner)



//Default Constructor
GAPlanner::GAPlanner()
{

}
GAPlanner::GAPlanner(ros::NodeHandle &nh)
{
  ROSNodeHandle = nh;

}

GAPlanner::GAPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  //OGM=new OccupancyGridMap();
  initialize(name, costmap_ros);
}

void GAPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

  if (!initialized_)
  {
    
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    ros::NodeHandle private_nh("~/" + name);

    originX = costmap_->getOriginX();
    originY = costmap_->getOriginY();

  width = costmap_->getSizeInCellsX();

  height = costmap_->getSizeInCellsY();

  resolution = costmap_->getResolution();

  mapSize = width*height;
  tBreak = 1+1/(mapSize); 
  value =0;


  OGM_data = (int **)malloc(sizeof(int *) * height);
  for (int i = 0; i < width; i++)
    OGM_data[i] = (int *)malloc(sizeof(int) * width);
  
    for (unsigned int iy = 0; iy < height; iy++)
    {
      for (unsigned int ix = 0; ix < width; ix++)
      {
        unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
        //cout<<cost;
        if (cost == 0)
          OGM_data[iy][ix]=0;
        else if (cost == 255)
          OGM_data[iy][ix]=-1;
        else if (cost == 254)
          OGM_data[iy][ix]=100;
        else if (cost == 253)
          OGM_data[iy][ix]=99;
        else
          OGM_data[iy][ix]=char(1+(97*(cost - 1))/251);
      }
    }

      OGM = new bool [mapSize]; 
    for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    {
      for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
      {
        unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
        //cout<<cost;
        if (cost == 0)
          OGM[iy*width+ix]=true;
        else
          OGM[iy*width+ix]=false;
      }
    }

    ROS_INFO("GA planner initialized successfully");
    initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized... doing nothing");
}

bool GAPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{

  if (!initialized_)
  {
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

  plan.clear();

  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
              costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }

  tf::Stamped < tf::Pose > goal_tf;
  tf::Stamped < tf::Pose > start_tf;

  poseStampedMsgToTF(goal, goal_tf);
  poseStampedMsgToTF(start, start_tf);

  // convert the start and goal positions

  float startX = start.pose.position.x;
  float startY = start.pose.position.y;

  float goalX = goal.pose.position.x;
  float goalY = goal.pose.position.y;

  getCorrdinate(startX, startY);
  getCorrdinate(goalX, goalY);

  int startCell;
  int goalCell;

  if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
  {
    startCell = convertToCellIndex(startX, startY);

    goalCell = convertToCellIndex(goalX, goalY);

  }
  else
  {
    ROS_WARN("the start or goal is out of the map");
    return false;
  }

  if (isStartAndGoalCellsValid(startCell, goalCell)){

        vector<int> bestPath;
  bestPath.clear();

    bestPath = GAplanner(startCell, goalCell);

    if ( bestPath.size()>0)
    {

      for (int i = 0; i < bestPath.size(); i++)
      {

        float x = 0.0;
        float y = 0.0;

        int index = bestPath[i];

        convertToCoordinate(index, x, y);

        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
      }


  float path_length = 0.0;
  
  std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
  
  geometry_msgs::PoseStamped last_pose;
  last_pose = *it;
  it++;
  for (; it!=plan.end(); ++it) {
     path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x, 
                     (*it).pose.position.y - last_pose.pose.position.y );
     last_pose = *it;
  }
  cout <<"The global path length: "<< path_length<< " meters"<<endl;
  //MyExcelFile << "\t" <<path_length <<"\t"<< plan.size() <<endl;
      //publish the plan

      return true;

    }

    else
    {
      ROS_WARN("The planner failed to find a path, choose other goal position");
      return false;
    }

  }

  else
  {
    ROS_WARN("Not valid start or goal");
    return false;
  }

}
void GAPlanner::getCorrdinate(float& x, float& y)
{

  x = x - originX;
  y = y - originY;

}

int GAPlanner::convertToCellIndex(float x, float y)
{

  int cellIndex;

  float newX = x / resolution;
  float newY = y / resolution;

  cellIndex = getCellIndex(newY, newX);

  return cellIndex;
}

void GAPlanner::convertToCoordinate(int index, float& x, float& y)
{

  x = getCellColID(index) * resolution;

  y = getCellRowID(index) * resolution;

  x = x + originX;
  y = y + originY;

}

bool GAPlanner::isCellInsideMap(float x, float y)
{
  bool valid = true;

  if (x > (width * resolution) || y > (height * resolution))
    valid = false;

  return valid;
}

void GAPlanner::mapToWorld(double mx, double my, double& wx, double& wy){
   costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    wx = costmap->getOriginX() + mx * resolution;
    wy = costmap->getOriginY() + my * resolution;
}

vector<int> GAPlanner::GAplanner(int startCell, int goalCell){
  Path* bestPath= new Path();
  int numberOfIterations = 15;
  uint populationSize = 15;
  int crossoverType = 1; // 1: one point, 2: two point, 3: modified.
  float crossoverProbability = 0.9;
  float mutationProbability = 0.01;
  int mutationIterationNumber = 50;
  float minInitialPathCost = 0;
  int radius = 2;
  GA *ga=new GA(numberOfIterations, populationSize, crossoverType, crossoverProbability, mutationProbability, mutationIterationNumber, minInitialPathCost, radius);
  OccupancyGridMap *OGM_t = new OccupancyGridMap(width, height, resolution, OGM_data, 1, 1);
   timespec time1;
  /* take current time here */
    Population* initialPopulation = new Population();
    initialPopulation = ga->getInitialPopulation(OGM_t, startCell,goalCell);
    if (initialPopulation->getPopulation().size() > 0) 
    {
       bestPath=ga->findPath(OGM_t, initialPopulation, time1);

    }
    else
      cout<<"Can not generate initial population, please choose different start or goal positions"<<endl;
    ROS_INFO("GA planner done successfully");

  return bestPath->getPath();

}

vector <int> GAPlanner::findFreeNeighborCell (int CellID){
 
  int rowID=getCellRowID(CellID);
  int colID=getCellColID(CellID);
  int neighborIndex;
  vector <int>  freeNeighborCells;

  for (int i=-1;i<=1;i++)
    for (int j=-1; j<=1;j++){
      //check whether the index is valid
     if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
  neighborIndex = getCellIndex(rowID+i,colID+j);
        if(isFree(neighborIndex) )
      freeNeighborCells.push_back(neighborIndex);
  }
    }
    return  freeNeighborCells;
 
}

/*******************************************************************************/
//Function Name: isStartAndGoalCellsValid
//Inputs: the start and Goal cells
//Output: true if the start and the goal cells are valid
//Description: check if the start and goal cells are valid
/*********************************************************************************/
bool GAPlanner::isStartAndGoalCellsValid(int startCell,int goalCell)
{ 
 bool isvalid=true;
 bool isFreeStartCell=isFree(startCell);
 bool isFreeGoalCell=isFree(goalCell);
    if (startCell==goalCell)
    {
    //cout << "The Start and the Goal cells are the same..." << endl; 
    isvalid = false;
    }
   else
   {
      if (!isFreeStartCell && !isFreeGoalCell)
      {
  //cout << "The start and the goal cells are obstacle positions..." << endl;
        isvalid = false;
      }
      else
      {
  if (!isFreeStartCell)
  {
    //cout << "The start is an obstacle..." << endl;
    isvalid = false;
  }
  else
  {
      if(!isFreeGoalCell)
      {
        //cout << "The goal cell is an obstacle..." << endl;
        isvalid = false;
      }
      else
      {
        if (findFreeNeighborCell(goalCell).size()==0)
        {
    //cout << "The goal cell is encountred by obstacles... "<< endl;
    isvalid = false;
        }
        else
        {
    if(findFreeNeighborCell(startCell).size()==0)
    {
      //cout << "The start cell is encountred by obstacles... "<< endl;
      isvalid = false;
    }
        }
      }
  }
      }
  }
 return isvalid;
}


 float  GAPlanner::getMoveCost(int i1, int j1, int i2, int j2){
   float moveCost=INFINIT_COST;//start cost with maximum value. Change it to real cost of cells are connected
   if((j2==j1+1 && i2==i1+1)||(i2==i1-1 && j2==j1+1) ||(i2==i1-1 && j2==j1-1)||(j2==j1-1 && i2==i1+1)){
     //moveCost = DIAGONAL_MOVE_COST;
     moveCost = 1.4;
   }
    //if cell 2(i2,j2) exists in the horizontal or vertical line with cell1(i1,j1)
   else{
     if ((j2==j1 && i2==i1-1)||(i2==i1 && j2==j1-1)||(i2==i1+1 && j2==j1) ||(i1==i2 && j2==j1+1)){
       moveCost = 1;
     }
   }
   return moveCost;
 } 
 
  float  GAPlanner::getMoveCost(int CellID1, int CellID2){
   int i1=0,i2=0,j1=0,j2=0;
    
   i1=getCellRowID(CellID1);
   j1=getCellColID(CellID1);
   i2=getCellRowID(CellID2);
   j2=getCellColID(CellID2);
    
    return getMoveCost(i1, j1, i2, j2);
 } 


 //verify if the cell(i,j) is free
 bool  GAPlanner::isFree(int i, int j){
   int CellID = getCellIndex(i, j);
 return OGM[CellID];

 } 

  //verify if the cell(i,j) is free
 bool  GAPlanner::isFree(int CellID){
 return OGM[CellID];
 } 
}
;

bool operator<(cells const &c1, cells const &c2) { return c1.fCost < c2.fCost; }

// PLUGINLIB_EXPORT_CLASS(GA_planner::GAPlanner, nav_core::BaseGlobalPlanner);