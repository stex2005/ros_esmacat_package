#include "interface.h"

int main(int argc, char **argv)
{

  //Initialize ROS
  ros::init(argc, argv, "EsmaCAT_interface");

  //Start a thread that will write cyclically data to hard real-time EsmaCAT software
  //boost::thread boost_ROS_publish_thread(ROS_publish_thread);

  //Start a thread that handles keyboard digits and writes data to hard real-time EsmaCAT software
  boost::thread boost_ROS_command_thread(ROS_command_thread);

  ROS_INFO("SmartBox ROS Interface node threads are instantiated...");

  //Declare a subscriber to listen to any data sent from the hard real-time EsmaCAT software
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("EsmaCAT_pub_ecat_ros", 1000, ROS_subscribe_callback);

  ros::spin();

  //Join threads once execution is complete
  ROS_INFO("Interface node threads are joining...");
  boost_ROS_command_thread.join();

  return 0;
}


void ROS_publish_thread(){
  //Declare a message and setup the publisher for that message
  esmacat_pkg::esmacat_command command;
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher pub_esmacat_write = n.advertise<esmacat_pkg::esmacat_command>("EsmaCAT_sub_ecat_ros",1000);

  //Variables that setup the publishing loop
  int interim_roscount = 0;
  int32_t encoder_count_per_turn = 1024;
  double commanded_position_in_degrees = 0.0; //Initialize command
  double command_period_in_seconds = 10;

  //SET ESCON MOTOR SWITCH TO START MOTOR OPERATION
  command.enable = 1; // Value = 1 implies activate the motor

  while (ros::ok()){

    //SPEED CONTROL SINUSOIDAL COMMAND -----------------------------------------------------------
    command.setpoint = 0.05*sin(1/command_period_in_seconds*(2.0*3.14159)*interim_roscount/100.0);
    //--------------------------------------------------------------------------------------------
    //Send data to the hard real-time loop
    pub_esmacat_write.publish(command);

    loop_rate.sleep();
    interim_roscount++;
  }

}

void ROS_command_thread(){


  //Initialize Robot status
  char c;
  string inputString;
  RobotState state(STOP);
  bool swap_state(false);

  //Declare a message and setup the publisher for that message
  esmacat_pkg::esmacat_command command;
  ros::NodeHandle n;
  ros::Publisher pub_esmacat_write = n.advertise<esmacat_pkg::esmacat_command>("EsmaCAT_sub_ecat_ros",1000);

  //SET ESCON MOTOR SWITCH TO START MOTOR OPERATION
  command.enable = 1; // Value = 1 implies activate the motor

  print_command_keys();

  while (ros::ok()){


    // Get character
    c = cin.get();

    if(c != '\n'){
      switch(c){

      case 's': case 'S':
        if (state != STOP)
        {
          std::cout << green_key << "Quick-swapped to STOP mode!" << color_key << std::endl;
          state = STOP;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in STOP mode" << color_key <<  std::endl;
        }
        break;
      case 'n': case 'N':
        if (state != NULLTORQUE)
        {
          std::cout << green_key << "Quick-swapped to NULL-TORQUE mode!" << color_key << std::endl;
          state = NULLTORQUE;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in NULL-TORQUE mode" << color_key <<  std::endl;
        }
        break;
      case 'g': case 'G':
        if (state != GRAVITY)
        {
          std::cout << green_key << "Quick-swapped to GRAVITY mode!" << color_key << std::endl;
          state = GRAVITY;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in GRAVITY mode" << color_key <<  std::endl;
        }
        break;
      case 'f': case 'F':
        if (state != FREEZE)
        {
          std::cout << green_key << "Quick-swapped to FREEZE mode!" << color_key << std::endl;
          state = FREEZE;
          swap_state = true;
        }
        else
        {
          std::cout << yellow_key << "Already in FREEZE mode" << color_key <<  std::endl;
        }
        break;
      case 'k': case 'K':

        if (state == STOP or state == EXIT)
        {   swap_state = true;
          state = EXIT;
          std::cout << yellow_key << "Ending program - no more inputs..." << color_key << std::endl;
        }
        else {
          std::cout << yellow_key << "First stop the motor" << color_key << std::endl;

        }
        break;
      case ' ':
        print_command_keys();
        break;
      default:
        std::cout << red_key << "Unrecognized key input!" << color_key <<  std::endl;
        break;
      }

      /**
                 * This is a message object. You stuff it with data, and then publish it.
                 */


      command.state = state;

      ROS_INFO("Publish State: %li", command.state);
      pub_esmacat_write.publish(command);

      ros::spinOnce();


      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    else
    {
      std::cout << yellow_key << state_labels[state] << color_key << " is the state currently active" << std::endl;
    } // else
  } // while
  ROS_INFO("ROS command thread stopped");

} // thread

void ROS_subscribe_callback(const esmacat_pkg::esmacat_sensor msg)
{
  //Display data from hard real-time loop to the the terminal.
      ROS_INFO(" Enc:[%i]",msg.encoder);
}

// Print commands on terminal
void print_command_keys()
{
  std::cout << boldred_key << "\nCOMMAND KEYS:"<< color_key << std::endl;
  std::cout << blue_key << "\'k\'" << color_key << ": exit" << "\n";
  std::cout << blue_key << "\'s\'" << color_key << ": STOP mode"<< "\n";
  std::cout << blue_key << "\'n\'" << color_key << ": NULL-TORQUE mode" << "\n";
  std::cout << blue_key << "\'g\'" << color_key << ": GRAVITY mode"<< "\n";
  std::cout << blue_key << "\'f\'" << color_key << ": FREEZE mode"<< "\n";
  std::cout << blue_key << "\'ENTER\'" << color_key << ": SHOW current settings and command keys\n"<< "\n";
}
