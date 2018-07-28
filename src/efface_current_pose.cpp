#include <efface_current_pose/efface_current_pose.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <math.h>


//This layer simply sets the cost in the robot's current location to be zero. This functions as a "wagon rut" costmap layer.
PLUGINLIB_EXPORT_CLASS(wagon_rut_costmap_namespace::EffaceLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;


namespace wagon_rut_costmap_namespace
{

// double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew){
//     double dx = x-x0, dy = y-y0;
//     double h = sqrt(dx*dx+dy*dy);
//     double angle = atan2(dy,dx);
//     double mx = cos(angle-skew) * h;
//     double my = sin(angle-skew) * h;
//     double f1 = pow(mx, 2.0)/(2.0 * varx),
//            f2 = pow(my, 2.0)/(2.0 * vary);
//     return A * exp(-(f1 + f2));
// }

double gaussian(double dist, double A, double varx, double vary, double mx, double my){
    // double dx = x-x0, dy = y-y0;
    double h = dist;
    // double angle = angle;
    // double mx = cos(angle-skew) * h;
    // double my = sin(angle-skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * varx),
           f2 = pow(my, 2.0)/(2.0 * vary);
    return A * exp(-(f1 + f2));
}

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
    // file.close();
    file.open("/home/strider/catkin_ws/src/wagon_rut_costmap/maps/filename.txt", ios::out | ios::trunc);
    file << "false";
    file.close();
  } else if (content == "false") {
    result = false;
  //   // file.close();
    file.open("/home/strider/catkin_ws/src/wagon_rut_costmap/maps/filename.txt", ios::out | ios::trunc);
    file << "true";
    file.close();
  } else {
    return true;
  }
  // file.close();
  return result;
  // return true;
}




EffaceLayer::EffaceLayer() {}

void EffaceLayer::onInitialize()
{
  ros::NodeHandle nh("~");
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&EffaceLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  // bool map_a;
  // nh.param("map_a", map_a, true);
  map_a = update_filename();
  // map_a = true;
  if (map_a) {
    file_name = "/home/strider/catkin_ws/src/wagon_rut_costmap/maps/map_a.pgm";
  } else {
    file_name = "/home/strider/catkin_ws/src/wagon_rut_costmap/maps/map_b.pgm";
  }
  // ROS_INFO_STREAM(file_name);
  moving = false;

  movement_sub = nh.subscribe("/mobile_base/commands/velocity", 1, &EffaceLayer::movement_callback, this);
  // nh.setParam("map_a", !map_a);

}

void EffaceLayer::movement_callback(const geometry_msgs::Twist& twist)
{
  if ((twist.linear.x == 0.0) && (twist.linear.y == 0.0) && (twist.linear.z == 0.0)) {
    moving = false;
  } else {
    moving = true;
  }
}

void EffaceLayer::matchSize() //This is a necessary function for every costmap layer
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void EffaceLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void EffaceLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
//This function sets the cost in this layer of the costmap to zero at the current location of the robot
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
    // ROS_INFO_STREAM("updating");
    int cost = getCost(mx, my);
    //ROS_INFO_STREAM(cost);
    setCost(mx, my, 20); //Sets the value to zero in this location on the layer


    for (int i = 2; i < 5; i++) {
      for (int t = 0; t < 360; t++) {
        double rad = t * 0.0174533;
        int x = mx + i * cos(rad);
        int y = my + i * sin (rad);
        if (t > 175 & t < 185) {
          setCost(x, y, 0);
        } else {
          // int a = gaussian(i, 1.0, 2.0, 2.0, mx, my);
        // ROS_INFO_STREAM(a);
        // ROS_INFO_STREAM(a);
          setCost(x, y, -4*i + 20);
          // setCost(x, y, a);
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

void EffaceLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
//This is where the change to the costs in the master map occurs
{
  if (!enabled_)
    return;
  //
  // if (!moving) {
  //   master_grid.saveMap(file_name);
  //   return;
  // }
  // ROS_INFO_STREAM("updating");
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
        master_grid.setCost(i, j, old_cost - costmap_[index]); //Sets the cost in the current location to 0
      }
    }
  }
  // if (map_a) {
  //   master_grid.saveMap("/home/strider/catkin_ws/src/simple_costmap_layer/world_files/map_a.pgm");
  // } else {
  //   master_grid.saveMap("/home/strider/catkin_ws/src/simple_costmap_layer/world_files/map_b.pgm");
  // }
  // moving = false;
  // ROS_INFO_STREAM(file_name);
  master_grid.saveMap(file_name);
}
}
