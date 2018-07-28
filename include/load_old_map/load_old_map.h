#ifndef LOAD_OLD_LAYER
#define LOAD_OLD_LAYER
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <fstream>
#include <array>
#include <iostream>     // std::cout
#include <sstream>      // std::istringstream

using namespace std;

vector<string> vector_of_costs();
bool update_filename_load();

namespace wagon_rut_costmap_namespace
{
  class LoadOldLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
  {
  public:
    LoadOldLayer();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                               double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    bool isDiscretized()
    {
      return true;
    }
    virtual void matchSize();

  private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    bool map_a;
    string file_name;

  };

}

#endif
