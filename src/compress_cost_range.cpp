#include <pluginlib/class_list_macros.h>
#include <compress_cost_range/compress_cost_range.h>


//This is the code for the layer that compresses the range of possible costs in the costmap.
PLUGINLIB_EXPORT_CLASS(wagon_rut_costmap_namespace::CompressLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;

namespace wagon_rut_costmap_namespace
{
  CompressLayer::CompressLayer() {}

  void CompressLayer::onInitialize()
  {
    ros::NodeHandle nh("~");
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&CompressLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void CompressLayer::matchSize() //Necessary function for all costmap layers
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }

    void CompressLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
      enabled_ = config.enabled;
    }

    void CompressLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
    //Updates bounds of area to be changed to be the entire map, since this layer compresses the costs across the map.
    {
      if (!enabled_)
        return;

      *min_x = std::min(*min_x, 0.0);
      *min_y = std::min(*min_y, 0.0);
      *max_x = std::max(*max_x, 200.0);
      *max_y = std::max(*max_y, 200.0);
    }

    void CompressLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    //This is the function where the actual compression happens on the master grid (which is the overall costmap)
    //Note: The true range of costs on a costmap is 0 to 255. I am compressing it to be 50 to 255. However, the costs ultimately display within the costmap in a further compressed range from 20 to 100.
    {
      if (!enabled_)
        return;

      for (int j = min_j; j < max_j; j++) //Iterates through the entire costmap
      {
        for (int i = min_i; i < max_i; i++)
        {
          int index = getIndex(i, j);
          int cost = ((205*master_grid.getCost(i, j))/255) + 50; //gets the previous cost of this location in the costmap, and calculates the compressed cost
          //int cost = std::max(costmap_[index], master_grid.getCost(i, j));
          master_grid.setCost(i, j, cost); //Sets the new cost in the current location in the costmap to be the compressed cost
        }
      }
    }
}
