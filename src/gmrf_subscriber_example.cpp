#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"

#include <tf/transform_listener.h>
#include "gas_map_msgs/GasGrid.h"
// global variables
gas_map_msgs::GasGrid GMRFmapData;
bool GMRFmapDataSubscribed = false;
tf::StampedTransform transform;
float map_origin_x = 0;
float map_origin_y = 0;
void gas_map_cb(const gas_map_msgs::GasGrid::ConstPtr& msg)
{
  GMRFmapData=*msg;
  GMRFmapDataSubscribed=true;
}

//gridValue function
float GMRFMeanValue(gas_map_msgs::GasGrid &map,std::vector<float> Xp)
{
      float resolution=map.m_resolution;
      float Xstartx=map.m_x_min+map_origin_x;
      float Xstarty=map.m_y_min+map_origin_y;

      float width=map.m_size_x;
      std::vector<double> Data=map.mean;

      //returns grid value at "Xp" location
      //map data:  100 occupied      -1 unknown       0 free
      float indx=(floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );
      double out;
      out=Data[int(indx)];
      return out;
}

//gridValue function
float GMRFVarianceValue(gas_map_msgs::GasGrid &map,std::vector<float> Xp)
{
      float resolution=map.m_resolution;
      float Xstartx=map.m_x_min+map_origin_x;
      float Xstarty=map.m_y_min+map_origin_y;

      float width=map.m_size_x;
      std::vector<double> Data=map.var;

      //returns grid value at "Xp" location
      //map data:  100 occupied      -1 unknown       0 free
      float indx=(floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );
      double out;
      out=Data[int(indx)];
      return out;
}

int main(int argc, char **argv)
{
  // generate the same numbers as in the original C test program
  ros::init(argc, argv, "gmrf_subscriber_example");
  ros::NodeHandle nh;
  bool use_slam;
  ros::param::param<bool>("use_slam", use_slam, true); 
  //---------------------------------------------------------------
  ros::Subscriber gas_map_sub= nh.subscribe("gmrf_node/gas_grid_map", 100 ,gas_map_cb);	

  ros::Rate rate(100); 
  while(!GMRFmapDataSubscribed)
  {
    ros::spinOnce();  
    ros::Duration(0.1).sleep();
  }
  bool tf_map_world = false;
  tf::TransformListener listener;

  if(use_slam)
  {
    while(!tf_map_world)
    {
      try
      {
        listener.lookupTransform("/world", "/map",ros::Time(0), transform);
        map_origin_x=transform.getOrigin().x();
        map_origin_y=transform.getOrigin().y();
        tf_map_world = true;
      }
      catch (tf::TransformException ex)
      {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
      }
    }
  }

  // Main loop
  while (ros::ok())
  {
    std::cout << "MAP INFO:" << GMRFmapData.m_resolution << " " << GMRFmapData.m_size_x << " " << GMRFmapData.m_size_y << " " << GMRFmapData.m_x_min<< " " << GMRFmapData.m_y_min << " " << GMRFmapData.m_x_max<< " " << GMRFmapData.m_y_max << std::endl;
    std::vector<float> map_point;
    map_point.push_back(2);
    map_point.push_back(3);
    std::cout << "Get GMRF at point (2,3)" << std::endl;
    std::cout << "Variance: " << GMRFVarianceValue(GMRFmapData,map_point) << std::endl;
    std::cout << "Mean: " << GMRFMeanValue(GMRFmapData,map_point) << std::endl;
    map_point.clear();
    map_point.push_back(-3);
    map_point.push_back(0);
    std::cout << "Get GMRF at point (-3,0)" << std::endl;
    std::cout << "Variance: " << GMRFVarianceValue(GMRFmapData,map_point) << std::endl;
    std::cout << "Mean: " << GMRFMeanValue(GMRFmapData,map_point) << std::endl;
    map_point.clear();
    map_point.push_back(0);
    map_point.push_back(0);
    std::cout << "Get GMRF at point (0,0)" << std::endl;
    std::cout << "Variance: " << GMRFVarianceValue(GMRFmapData,map_point) << std::endl;
    std::cout << "Mean: " << GMRFMeanValue(GMRFmapData,map_point) << std::endl;    
    map_point.clear();
    map_point.push_back(0);
    map_point.push_back(6);
    std::cout << "Get GMRF at point (0,6)" << std::endl;
    std::cout << "Variance: " << GMRFVarianceValue(GMRFmapData,map_point) << std::endl;
    std::cout << "Mean: " << GMRFMeanValue(GMRFmapData,map_point) << std::endl; 
    ros::spinOnce();
    rate.sleep();
    ros::Duration(1.0).sleep();
  }
  return 0;
}
