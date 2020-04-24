#include "interface.h"

int main(int argc, char **argv)
{

  //Initialize ROS
  ros::init(argc, argv, "EsmaCAT_interface");

  smartbox_interface smartxbox_ros;

  ros::MultiThreadedSpinner spinner(1); // Use 4 threads
  spinner.spin();

  return 0;
}
