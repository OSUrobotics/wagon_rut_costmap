#include <efface_current_pose/efface_current_pose.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <math.h>


//This layer  sets the cost in the robot's current location to be zero. This functions as a "wagon rut" costmap layer.
//It also saves the current costmap to a file (alternating between map_a and map_b) so that it can be reloaded on the next launch by the load_old_map layer.
PLUGINLIB_EXPORT_CLASS(wagon_rut_costmap_namespace::EffaceLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;


namespace wagon_rut_costmap_namespace
{

//This function determines whether the costmap will be saved as map_a or map_b, so that it alternates each time the costmap is launched
//Update the location of the file to a stable location where you want to save data
bool update_filename()
{
  bool result;
  fstream file;
  file.open("/home/strider/catkin_ws/src/wagon_rut_costmap/maps/filename.txt", ios::in);
  string content;

  file >> content;
  file.close();
  if (content == "true") {
    result = true;
    file.open("/home/strider/catkin_ws/src/wagon_rut_costmap/maps/filename.txt", ios::out | ios::trunc);
    file << "false";
    file.close();
  } else if (content == "false") {
    result = false;
    file.open("/home/strider/catkin_ws/src/wagon_rut_costmap/maps/filename.txt", ios::out | ios::trunc);
    file << "true";
    file.close();
  } else {
    return true;
  }
  return result;
}



//This is the class containing the actual wagon rut costmap layer
EffaceLayer::EffaceLayer() {}

void EffaceLayer::onInitialize()
{
  //This code is necessary for all costmap layers
  ros::NodeHandle nh("~");
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&EffaceLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  //The following block of code sets up the system to save the costmap to alternating locations upon each launch
  map_a = update_filename();
  if (map_a) {
    file_name = "/home/strider/catkin_ws/src/wagon_rut_costmap/maps/map_a.pgm";
  } else {
    file_name = "/home/strider/catkin_ws/src/wagon_rut_costmap/maps/map_b.pgm";
  }

  //This variable determines whether the robot is moving, so that the robot will not create wagon ruts when it is still
  moving = false;
  //It then subscribes to the velocity topic
  movement_sub = nh.subscribe("/mobile_base/commands/velocity", 1, &EffaceLayer::movement_callback, this);

}

//Updates moving variable
void EffaceLayer::movement_callback(const geometry_msgs::Twist& twist)
{
  if ((twist.linear.x == 0.0) && (twist.linear.y == 0.0) && (twist.linear.z == 0.0)) {
    moving = false;
  } else {
    moving = true;
  }
}

//This is a necessary function for every costmap layer
void EffaceLayer::matchSize() 
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

//This is a necessary function for every costmap layer
void EffaceLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}
    
//This function sets the cost in this layer of the costmap to 20 at the current location of the robot, with a linearly decreasing cost around the robot's point.
//It also changes the bounds being updated in the costmap to be the entire map.
void EffaceLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if (!moving)
    return;

  unsigned int mx;
  unsigned int my;
  double mark_x = robot_x, mark_y = robot_y; //The mark is at the robot's current location
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();

  if(worldToMap(mark_x, mark_y, mx, my)){ //Checks that the location to change is a valid location on the map
    int cost = getCost(mx, my);
    setCost(mx, my, 20); //Sets the value to 20 in this location on the layer


    for (int i = 2; i < 5; i++) {
      for (int t = 0; t < 360; t++) {
        double rad = t * 0.0174533;
        int x = mx + i * cos(rad);
        int y = my + i * sin (rad);
        if (t > 175 & t < 185) {
          setCost(x, y, 0);
        } else {
          setCost(x, y, -4*i + 20); //Sets up a decreasing cost radiating out from the robot starting from 20 at the point of the robot
        }
      }
    }
  }

  //Currently just updating the bounds to change to be the entire map
  *min_x = std::min(*min_x, 0.0);
  *min_y = std::min(*min_y, 0.0);
  *max_x = std::max(*max_x, 200.0);
  *max_y = std::max(*max_y, 200.0);

}

//This is where the change to the costs in the master map occurs
void EffaceLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++) //Iterates through the entire costmap
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION) { //Makes sure that the master costmap is only updated if there was an update in this layer
        continue;
      }

      int old_cost = master_grid.getCost(i, j);
      if ((old_cost - costmap_[index]) < 0) {
        continue;
      } else if (old_cost > LETHAL_OBSTACLE - 20) {
        continue;
      } else {
        // master_grid.setCost(i, j, 0);
        master_grid.setCost(i, j, old_cost - costmap_[index]); //Sets the cost in the current location to the previous cost in this location, minus the cost set for this layer in the updateBounds method
      }
    }
  }
  master_grid.saveMap(file_name);
}
}
