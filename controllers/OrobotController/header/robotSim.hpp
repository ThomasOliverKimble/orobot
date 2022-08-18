#ifndef ROBOTSIM_HPP
#define ROBOTSIM_HPP


#define OPTIMIZATION
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <sys/time.h>
#include "joystick.h"
#include "utils.hpp"
#include <fstream>
#include <iostream>
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Gyro.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Receiver.hpp>
#include <webots/Compass.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include "misc_math.hpp"

#define N_TOUCH_SENSORS    4
#define N_anTr      1000





using namespace std;
using namespace Eigen;
//using namespace Robot;
//using Eigen::Matrix Matrix;



class RobotSim : public webots::Supervisor{


  public:
    //================== public variables ===============================================
    vector<webots::Motor*> rm_motor;
    vector<webots::PositionSensor*> ps_motor;
    webots::Gyro *gyro;
    webots::Accelerometer *acc;
    webots::Compass *compass;
    webots::GPS *gps;
    webots::TouchSensor *touch_sensor[N_TOUCH_SENSORS];
    webots::Receiver *rec;


    webots::TouchSensor *touch_sensor_spine[12];
    webots::Node *fgirdle, *FL_marker, *FR_marker, *HL_marker, *HR_marker, *CoM_marker, *roboDef, *tsdefFL, *tsdefFR, *tsdefHL, *tsdefHR;
    const double *compassData, *gpsData, *gyroData, *accData, *ts_fl, *ts_fr, *ts_hl, *ts_hr, *rotMat, *posFL, *posFR, *posHL, *posHR;
    webots::Field *roboRot, *roboPos, *CoM_marker_pos;
    double gamma, tcyc, tcyc_an;
    double t_total;

    // ======================= FORCE PLATES =================================================
    webots::TouchSensor *forcePlates[4];
    webots::Node *forcePlateHolder[4];

    //================== public functions ===============================================
    RobotSim(int TIME_STEP); // constructor
    void setAngles(double*,int*);
    void ReadSensors(double *d_posture, double *d_torques, double *gyroData, double *accData, double *gpsData);
    void GetGPS(double *gpsData);
    void InitIMU();
    void getGRF(double *GRF);
    void ReadIMUAttitude(double *rotmat);
    void ReadTouchSensors(double *ts_data);
    void killSimulation();
    void ColorMarkers(double *logic, double trans, double *col1);
    void setPositionRotation(double *p, double *r);
    void setCoMmarker(double *p);
    void GetCompass(double *data_i);
    void GetFeetGPS(double *FL_feet_gpos, double *FR_feet_gpos, double *HL_feet_gpos, double *HR_feet_gpos);
    void setServoMaxForce(double *force);
    void GetTailForce(double *tailForce);
    void getTorqueFeedback(double *d_torques);
    void getPositionFeedback(double *d_posture);
    void getAngles(double *table);
    void set2segFeetParam(double spring1, double spring2);
    void startVideoRecording(const string& filename, int width, int height);
    void stopVideoRecording();
  private:
    //================== private variables ===============================================




};


#endif

