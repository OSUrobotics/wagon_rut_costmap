#include <pluginlib/class_list_macros.h>
#include <load_old_map/load_old_map.h>


PLUGINLIB_EXPORT_CLASS(wagon_rut_costmap_namespace::LoadOldLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;

// bool map_a;

int string_to_int(string str)
{
  if (str == "100") {
    return 100;
  }
  else if (str.length() == 2) {
    return 10*(str[0]-'0') + (str[1] - '0');
  } else {
    return 1*(str[0] - 0);
  }
}

bool update_filename_load()
{
  bool result;
  fstream file;
  // ofstream file_out;
  file.open("/home/strider/catkin_ws/src/wagon_rut_costmap/maps/filename.txt", ios::in);
  string content;

  file >> content;
  file.close();
  if (content == "true") {
    result = true;
    // file.close();
  } else if (content == "false") {
    result = false;
    // file.close();
  } else {
    return true;
    // file.close();
  }
  return result;
}

vector<string> vector_of_costs(bool map_a, string file_name)
{
  vector<string> costs;
  // costs.push_back("hey");
  ifstream file; //Change to be pgm file (old_map.pgm)
  // if (map_a)
  // {
  //   file.open("/home/strider/catkin_ws/src/simple_costmap_layer/world_files/map_a.pgm");
  // } else {
  //   file.open("/home/strider/catkin_ws/src/simple_costmap_layer/world_files/map_b.pgm");
  // }
  file.open(file_name);
  // int number;
  std::string content( (std::istreambuf_iterator<char>(file) ),
                       (std::istreambuf_iterator<char>()    ) );
  string num;
  // int i = 0;
  stringstream str(content);
  while (!str.eof()) //Hello. I think I might be stuck in an infinite loop. Hello. I think I might be stuck in an infinite loop. Hello. I think I might be stu....
  {
    // i++;
  // //   // getline(str, num, "\0");
  // //   // number = string_to_int(line);
    str >> num;
    costs.push_back(num);
  }
  // ROS_INFO_STREAM(costs[0]);
  file.close();
  return costs; //should return vector of strings split by spaces, hopefully including new lines
}

array<array<int, 200>, 200> array_of_costs(vector<string> vect)
{
  array<array<int, 200>, 200> arr;
  // int arr[200][200];
  int row = 0;
  int col = 0;
  for (int i = 0; i < vect.size(); i++)
  {
    // ROS_INFO_STREAM(vect.size());
    arr[row][col] = stoi(vect.at(i));

    if (col == 199) {
      row = row + 1;
      col = 0;
    } else {
      col = col + 1;
    }
    // if ((row < 200) && (col < 200)) {
    //   arr[row][col] = stoi(vect.at(i));
    // }
  }

  return arr;
}


namespace wagon_rut_costmap_namespace
{
  LoadOldLayer::LoadOldLayer() {}
  void LoadOldLayer::onInitialize()
  {
    ros::NodeHandle nh("~");
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&LoadOldLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    // bool map_a;
    // nh.param("map_a", map_a, true);
    map_a = update_filename_load();
    if (map_a) { //change back to map_a
      file_name = "/home/strider/catkin_ws/src/wagon_rut_costmap/maps/map_b.pgm";
    } else {
      file_name = "/home/strider/catkin_ws/src/wagon_rut_cstmap/maps/map_a.pgm";
    }
    ROS_INFO_STREAM(file_name);
  }

  void LoadOldLayer::matchSize() //Necessary function for all costmap layers
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }

  void LoadOldLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void LoadOldLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
  //Updates bounds of area to be changed to be the entire map
  {
    if (!enabled_)
      return;

    *min_x = std::min(*min_x, 0.0);
    *min_y = std::min(*min_y, 0.0);
    *max_x = std::max(*max_x, 200.0);
    *max_y = std::max(*max_y, 200.0);
  }

  void LoadOldLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    if (!enabled_)
      return;

    // if (!update){
    //   return;
    // }

    vector<string> old_cost_vector = vector_of_costs(map_a, file_name);
    // ROS_INFO_STREAM(old_cost_vector[0]);
    vector<string>::const_iterator first = old_cost_vector.begin() + 4;
    vector<string>::const_iterator last = old_cost_vector.end() - 1;
    vector<string> new_vec(first, last);
    array<array<int, 200>, 200> old_costs = array_of_costs(new_vec);
    // ROS_INFO_STREAM(new_vec.size());
    //WHY ARE YOU SHIFTING BY 1 EACH TIME??????


    for (int j = min_j; j < max_j; j++) //Iterates through the entire costmap
    {
      for (int i = min_i; i < max_i; i++)
      {
        // if (old_costs[j][i]) {
          int cost = old_costs[j][i];
          // ROS_INFO_STREAM(cost);
          master_grid.setCost(i, j, cost);
        // }
        // else {
          // master_grid.setCost(i, j, 0);
        // }
        // if (old_costs.size() > 0) {
        //   int index = getIndex(i, j);
        //   int cost = old_costs.front();
        //   old_costs.erase(old_costs.begin());
        //   if (cost >= 0 & cost <= 255) {
        // //     master_grid.setCost(i, j, cost); //Try to actually save costmap2d and load that back in somehow? :O
        //   }
        // } else {
        //   setCost(i, j, 0);
        // }
        // int cost = ((205*master_grid.getCost(i, j))/255) + 50; //gets the previous cost of this location in the costmap, and calculates the compressed cost
        //int cost = std::max(costmap_[index], master_grid.getCost(i, j));
        // master_grid.setCost(i, j, 0); //Sets the new cost in the current location in the costmap to be the compressed cost
      }
    }
    // update = false;
  }
}
