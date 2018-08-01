# include "ros/ros.h"
# include "std_msgs/String.h"
# include "geometry_msgs/Twist.h"
# include "geometry_msgs/TwistStamped.h"
# include "sensor_msgs/LaserScan.h"
# include "message_filters/time_synchronizer.h"
# include "message_filters/subscriber.h"
# include "std_msgs/Header.h"

using namespace std_msgs;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace message_filters;



class Circle
{
  public:
    ros::Publisher move_pub;
    ros::Subscriber scan_sub;

    Circle(ros::NodeHandle& n);
    //void teleop_callback(const Twist& input);
    //void scan_and_teleop_callback(const TwistStamped& input, const LaserScan& scan);
    void scan_callback(const LaserScan& scan);

  private:
    ros::NodeHandle n_;
};

Circle::Circle(ros::NodeHandle& n) : n_(n)
{
  // ROS_INFO_STREAM("got to 62");
  move_pub = n.advertise<Twist>("/mobile_base/commands/velocity", 1000);

  //ros::Publisher altered_input_pub = n.advertise<TwistStamped>("/my_altered_input", 1000);
  // ros::Subscriber teleop_sub = n.subscribe("/my_teleop", 1, &Teleop_and_Stop::teleop_callback, this);
  scan_sub = n.subscribe("/scan", 1, &Circle::scan_callback, this);

  // message_filters::Subscriber <LaserScan> scan_sub(n, "/scan", 1000);
  // message_filters::Subscriber <TwistStamped> altered_input_sub(n, "/my_altered_input", 1000);
  // TimeSynchronizer<TwistStamped, LaserScan> ats(altered_input_sub, scan_sub, 10);
  // ats.registerCallback(&Teleop_and_Stop::scan_and_teleop_callback);
}
//
// void Teleop_and_Stop::teleop_callback(const Twist& input)
// {
//   //if you like it you should put a stamp on it
//   TwistStamped output = TwistStamped();
//   output.twist = input;
//   output.header = Header();
//   output.header.stamp = ros::Time::now();
//   altered_input_pub.publish(output);
// }

void Circle::scan_callback(const LaserScan& scan)
{
  Twist cir = Twist();
  cir.linear.x = 5;
  cir.angular.z = 0.4;
  move_pub.publish(cir);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "circle");
  // ROS_INFO_STREAM("here");
  ros::NodeHandle n;
  Circle c(n);
  ros::spin();
  return 0;
}
