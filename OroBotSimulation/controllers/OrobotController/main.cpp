 
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Gyro.hpp>
#include <webots/Camera.hpp>
#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <ctime>
#include <fstream>
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include "controller.hpp"
#include "robotSim.hpp"
//#include "OptiCtrl.hpp"
#include "joystick.h"


#define TIME_STEP   10
#define MAX_NUM_MOTORS   40 

// load global config
int readGlobalConfig();


using namespace std;
using namespace Eigen;
using namespace webots;
int joystick_numbers = 1;




// GLOBAL CONFIG
int IS_SIMULATION;
int IS_OPTIMIZATION;
int USE_JOYSTICK;
int SPINE_COMPENSATION_WITH_FEEDBACK;
int USE_IMU;
int USED_MOTORS[50];
int AUTO_RUN, AUTO_RUN_STATE;
int LOG_DATA, RECORD_VIDEOS;
int ANIMAL_DATASET;
int JOYSTICK_RECORDING_REPLAYING;
int USE_FEET;
int SKIP_WRIST_SERVOS;
int NUM_MOTORS;
int SPINE_SIZE;


//############################ PARSE PARAMETER MESSAGES ###############################################



//############################ MAIN PROGRAM ###########################################################

int main(int argc, char **argv)
{
    if(readGlobalConfig()==0){
        return 0;
    }
    USE_FEET=USE_IMU;
    SKIP_WRIST_SERVOS=!USE_FEET;

    cout<<"MAIN STARTS"<<endl;

    double Td = TIME_STEP/1000.;
    double table_p[MAX_NUM_MOTORS], table_t[MAX_NUM_MOTORS];


    //======================================================//

    RobotSim robotSim(TIME_STEP);
    cout<<"ROBOT CREATED"<<endl;



    //cout << "creating controller" << endl;
    Controller controller(Td);

    cout<<"CONTROLLER CREATED"<<endl;


    //==========================================================================
    controller.initOrobot();

    double t=0;
    int k=1;

    double dt;
    int numsteps=0;

    VectorXd torques(NUM_MOTORS), position(NUM_MOTORS);
    double d_posture[MAX_NUM_MOTORS], d_torques[MAX_NUM_MOTORS], gyroData[3], accData[3], compassData[3], RPY[3], gpsDataF[3], gpsDataH[3], ts_data[12+12], markers[4];
    double fgird_rotMat[9], gpsData[3], GRF[12];
    double FL_feet_gpos[3], FR_feet_gpos[3], HL_feet_gpos[3], HR_feet_gpos[3];
    int count=0;


    //================================== FREQ DEPENDANT ================================
    //robotSim.set2segFeetParam(2.5, 2.5*0.25); // 0.5hz
    robotSim.set2segFeetParam(3.5, 3.5*0.25); // 0.75hz



cout << "STARTING THE LOOP" << endl;
//=============================  LOOP  =============================================
    while(robotSim.step(TIME_STEP) != -1) {

        const std::string text = robotSim.wwiReceiveText();
        if (!text.empty())
          cout << "MESSAGE RECEIVED: " << text << endl;

        // parse parameter string
        controller.parseParameterString(text);

        // restart sim
        size_t found; 
        found = text.find("restart");
        if (found!=string::npos){
            cout << "RESTARTING SIMULATION" << endl;
            robotSim.resetOrobotSimulation();
        }
        
        dt=Td;
        t=t+dt;
        controller.setTimeStep(dt);


        //read from robot
        robotSim.getPositionFeedback(d_posture);        //joint positions
        robotSim.getTorqueFeedback(d_torques);       // joint torques
        robotSim.ReadIMUAttitude(fgird_rotMat); //front girdle rotation matrix
        robotSim.GetGPS(gpsData);          // front girdle position
        robotSim.GetFeetGPS(FL_feet_gpos, FR_feet_gpos, HL_feet_gpos, HR_feet_gpos);
        robotSim.getGRF(GRF);


        controller.updateRobotState(d_posture, d_torques);
        controller.getAttitude(fgird_rotMat);
        controller.getGPS(gpsData, FL_feet_gpos, FR_feet_gpos, HL_feet_gpos, HR_feet_gpos);
        controller.GRFfeedback(GRF);

        if(!controller.runStep()){
            break;
        }
        //controller.globalKinematics(gpsData, fgird_rotMat);
        controller.getAngles(table_p);

        robotSim.setAngles(table_p, USED_MOTORS);



        //================================== LOG DATA ==================================================//
        if(LOG_DATA){


            static ofstream fullBodyKinematicsLog("./data/fullBodyKinematicsLog.txt");
            static ofstream timeLog("./data/timeLog.txt");

            // TIME
            timeLog<<t<<endl;


            // FORWARD KINEMATICS
            for(int i=0;i<SPINE_SIZE;i++){
                fullBodyKinematicsLog << controller.HJs_g[i].block<3,1>(0,3).transpose() << "\t";
            }
            for(int i=0;i<5;i++){
                fullBodyKinematicsLog << controller.HJfl_g[i].block<3,1>(0,3).transpose() << "\t";
            }
            for(int i=0;i<5;i++){
                fullBodyKinematicsLog << controller.HJfr_g[i].block<3,1>(0,3).transpose() << "\t";
            }
            for(int i=0;i<5;i++){
                fullBodyKinematicsLog << controller.HJhl_g[i].block<3,1>(0,3).transpose() << "\t";
            }
            for(int i=0;i<5;i++){
                fullBodyKinematicsLog << controller.HJhr_g[i].block<3,1>(0,3).transpose() << "\t";
            }
            fullBodyKinematicsLog << controller.legPhase.transpose() << endl;
            //fullBodyKinematicsLog << endl;




        }



        //==============================================================================


    }


  return 0;
}



// load global config
int readGlobalConfig()
{
    stringstream stringstream_file;
    ifstream global_config_file("config/GLOBAL.config");
    if(global_config_file.is_open()){
        readFileWithLineSkipping(global_config_file, stringstream_file);
        stringstream_file >> IS_SIMULATION;
        stringstream_file >> IS_OPTIMIZATION;
        stringstream_file >> USE_JOYSTICK;
        stringstream_file >> SPINE_COMPENSATION_WITH_FEEDBACK;
        stringstream_file >> USE_IMU;
        stringstream_file >> AUTO_RUN;
        stringstream_file >> AUTO_RUN_STATE;
        stringstream_file >> LOG_DATA;
        stringstream_file >> RECORD_VIDEOS;
        stringstream_file >> NUM_MOTORS;
        stringstream_file >> SPINE_SIZE;

        // IDs
        cout << "USED_IDS: \n";
        for(int i=0; i<NUM_MOTORS; i++){
            stringstream_file >> USED_MOTORS[i];

        }
        stringstream_file >> ANIMAL_DATASET;
        return 1;
    }

    else{
        return 0;
    }
}
