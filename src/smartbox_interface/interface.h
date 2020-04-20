#ifndef INTERFACE_H
#define INTERFACE_H

#include <iostream>
#include <thread>
#include <chrono>
#include "ros/ros.h"
#include "esmacat_pkg/esmacat_sensor.h"
#include "esmacat_pkg/esmacat_command.h"
#include <boost/thread/thread.hpp>

using namespace std;

// Text Color Identifiers
const string boldred_key = "\033[1;31m";
const string red_key = "\033[31m";
const string boldpurple_key = "\033[1;35m";
const string yellow_key = "\033[33m";
const string blue_key = "\033[36m";
const string green_key = "\033[32m";
const string color_key = "\033[0m";

//Labels for states
const string state_labels[] = {
    "EXIT",
    "STOP",
    "NULLTORQUE",
    "GRAVITY",
    "FREEZE",
    "QUIT",
};

void print_command_keys();
void ROS_publish_thread();
void ROS_command_thread();
void ROS_subscribe_callback(const esmacat_pkg::esmacat_sensor msg);

enum RobotState
{
    EXIT,
    STOP,
    NULLTORQUE,
    GRAVITY,
    FREEZE,
    QUIT,
};



#endif // INTERFACE_H
