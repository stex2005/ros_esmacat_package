#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <sstream>
#include <math.h>

#include "esmacat_shared_memory_comm.h"
//#include "robot.h"

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

#endif // USER_INTERFACE_H

