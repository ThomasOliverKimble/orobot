#ifndef OPTICTRL_HPP
#define OPTICTRL_HPP


#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <sys/time.h>
#include "joystick.h"
#include <fstream>
#include <iostream>
#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include <fstream>
#include <pthread.h>
#include <vector>
#include <string.h>
#include "robotSim.hpp"
#include "controller.hpp"
#include <optimization/webots.hh>
#include <sstream>
#include <stdlib.h>
//#define pi          3.141592653589793


using namespace optimization::messages::task;
using namespace std;

using namespace Eigen;
//using namespace Robot;
//using Eigen::Matrix Matrix;



class OptiCtrl{


  public:
    //================== public variables ===============================================
    std::vector<double> params;
    std::vector<double> settings;
    std::vector<std::string> params_names;
    std::vector<std::string> settings_names;
    double rolling, pitching, yawing, applied_torques, applied_torques_l, applied_torques_s;
    double t_total;
    double fitfun[20];
    optimization::Webots &opti = optimization::Webots::Instance();
    //================== public functions ===============================================
    OptiCtrl(); // constructor
    int OptiStep(double dt, Controller *controller, RobotSim *pleuro);
    void OptiLog(Controller *controller, RobotSim *pleuro);
    void optimizationInit(Controller *controller, RobotSim *pleuro);
    void optimizationEnd(RobotSim *pleuro);
    int optimizationShouldWeStopIt(double timestep);
  private:
    //================== private variables ===============================================




};


#endif