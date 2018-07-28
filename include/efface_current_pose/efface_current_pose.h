#ifndef EFFACE_LAYER
#define EFFACE_LAYER
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <fstream>
#include <iostream>
#include <geometry_msgs/Twist.h>

using namespace std;

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
bool update_filename();


namespace wagon_rut_costmap_namespace
{

class EffaceLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  EffaceLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }
  ros::Subscriber movement_sub;

  virtual void matchSize();
  bool map_a;
  std::string file_name;
  bool moving;
  void movement_callback(const geometry_msgs::Twist& twist);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
