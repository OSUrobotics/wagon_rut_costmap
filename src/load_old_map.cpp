#include <pluginlib/class_list_macros.h>
#include <load_old_map/load_old_map.h>

//This layer loads the previously saved costmap back in when the move_base node is launched.
PLUGINLIB_EXPORT_CLASS(wagon_rut_costmap_namespace::LoadOldLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;

const int side_of_map = 200;


//Updates which map file should be loaded based off of whether the map was last saved as map_a or map_b
bool update_filename_load()
{
  bool result;
  fstream file;
  file.open("/home/strider/catkin_ws/src/wagon_rut_costmap/maps/filename.txt", ios::in); //Update the file path to match the file path you use in the effacement layer
  string content;
  file >> content;
  file.close();
  if (content == "true") {
    result = true;
  } else if (content == "false") {
    result = false;
  } else {
    return true;
  }
  return result;
}

//Reads the previously saved pgm file into a vector of strings representing the saved costs from the costmap
vector<string> vector_of_costs(bool map_a, string file_name)
{
  vector<string> costs;
  ifstream file;
  file.open(file_name);
  std::string content( (std::istreambuf_iterator<char>(file) ),
                       (std::istreambuf_iterator<char>()    ) );
  string num;
  stringstream str(content);
  while (!str.eof())
  {
    str >> num;
    costs.push_back(num);
  }
  file.close();
  return costs; //returns vector of strings read in from the file, split by whitespace
}

//Takes in a vector of strings representing integers (such as the kind of vector returned by the above function) and converts it to a 200x200 array of integers
array<array<int, side_of_map>, side_of_map> array_of_costs(vector<string> vect)
{
  array<array<int, side_of_map>, side_of_map> arr;
  int row = 0;
  int col = 0;
  for (int i = 0; i < vect.size(); i++)
  {
    arr[row][col] = stoi(vect.at(i));
    if (col == side_of_map - 1) {
      row = row + 1;
      col = 0;
    } else {
      col = col + 1;
    }
  }
  return arr;
}


namespace wagon_rut_costmap_namespace
{
  //This is the class for the load old map layer
  LoadOldLayer::LoadOldLayer() {}
  void LoadOldLayer::onInitialize()
  {
    //This code is necessary for all costmap layers
    ros::NodeHandle nh("~");
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&LoadOldLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    // This block of code determines which file the costmap is read in from
    // map_a = update_filename_load();
    // if (map_a) {
    //   file_name = "/home/strider/catkin_ws/src/wagon_rut_costmap/maps/map_b.pgm";
    // } else {
    //   file_name = "/home/strider/catkin_ws/src/wagon_rut_costmap/maps/map_a.pgm";
    // }

    if (nh.getParam("costmap_path", file_name)) {
      ROS_INFO_STREAM(file_name);
    } else {
      ROS_INFO_STREAM("Life: not so good afterall.");
    }
  }

  //Necessary function for all costmap layers
  void LoadOldLayer::matchSize()
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }

  //Necessary function for all costmap layers
  void LoadOldLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  //Updates bounds of area to be changed to be the entire map
  void LoadOldLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if (!enabled_)
      return;

    // *min_x = std::min(*min_x, 0.0);
    // *min_y = std::min(*min_y, 0.0);
    // *max_x = std::max(*max_x, 200.0);
    // *max_y = std::max(*max_y, 200.0);
  }

  void LoadOldLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    if (!enabled_)
      return;

    vector<string> old_cost_vector = vector_of_costs(map_a, file_name);
    vector<string>::const_iterator first = old_cost_vector.begin() + 4;
    vector<string>::const_iterator last = old_cost_vector.end() - 1;
    vector<string> new_vec(first, last);
    // side_of_map = sqrt(new_vec.size());
    array<array<int, side_of_map>, side_of_map> old_costs = array_of_costs(new_vec);


    for (int j = min_j; j < max_j; j++) //Iterates through the entire costmap
    {
      for (int i = min_i; i < max_i; i++)
      {
          int cost = old_costs[j][i];
          master_grid.setCost(i, j, cost); //sets the cost to be the cost in the equivalent location in the old costmap
      }
    }
  }
}
