#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP



//#define EIGEN_DONT_ALIGN_STATICALLY
//#define EIGEN_DONT_VECTORIZE
//#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/SVD"
#include "joystick.h"
#include "utils.hpp"
#include "misc_math.hpp"
#include <fstream>
#include <pthread.h>
#include <vector>
#include <string.h>
#include <thread>  
#include <mutex> 
/////////////////////////////////
#include "dlib/optimization.h"
#include "dlib/optimization/find_optimal_parameters.h"
#include "qpOASES.hpp"


#define my_pi          3.141592653589793

#define MAX_NUM_MOTORS    40
#define N_anTr      1000
#define JOYSTICK_DEVNAME "/dev/input/js0"
/*
#define USE_JOYSTICK   1
#define JOYSTICK_TYPE 1     //PS1 - 1, PS3 - 2, Logitech - 3
#define SWIM   0
#define IS_SIMULATION 1
#define SPINE_COMPENSATION_WITH_FEEDBACK 0
*/

extern int IS_SIMULATION, USE_JOYSTICK, SPINE_COMPENSATION_WITH_FEEDBACK;
extern int USE_IMU, AUTO_RUN, AUTO_RUN_STATE, LOG_DATA, NUM_MOTORS, SPINE_SIZE;

using namespace Eigen;
//using namespace Robot;
//using Eigen::Matrix Matrix;
enum{WALKING, STANDING, POSING, SWIMMING, ANIMAL_WALKING, ANIMAL_SWIMMING, ANIMAL_AQUASTEP, INITIAL, NONPERIODIC};



class Controller{


  public:
    //================== public variables ===============================================
    Joystick js;
    int state;
    double gamma, freq_swim, freq_walk, freq_aquastep;
    Matrix<double, Dynamic, 1> fbck_torques, fbck_position, joint_angles;
    Matrix<double,12, 1> force, force_filt;
    Matrix<double,3, 1> posture;
    Matrix<double, 4, 1> legs_stance;
    double compassData[3], gpsData[3], gyroData[3], accData[3];
    Matrix<double, 3, Dynamic> global_joint_pos;
    Matrix<double, 3, 4> global_feet_pos;
    Matrix<double, 3, 4> feet_force, fc_feet_force;
    vector<Matrix4d> HJs, HJs_g;
    vector<Matrix4d> HJfl, HJfr, HJhl, HJhr, HJfl_g, HJfr_g, HJhl_g, HJhr_g;
    vector<MatrixXd> legJacob;
    Matrix<double, 4, 4> Fgird, Hgird, Fgird0, Hgird0, Fgird_mean, Hgird_mean, HJh, HJh_g;
    double hgirdlePhi;
    Vector4d phaseTransitionStanceSwing, phaseTransitionSwingStance, legs_stance_old;
    double IG;
    /*Matrix<double, 2, 1> fgird_pos, hgird_pos;
    double fgird_rot, hgird_rot;*/

    // needed for RUN
    Matrix<double, Dynamic, 1> spine_kin;
    Matrix<double, 2, 1> girdLoc;
    Matrix<double, 2, 1> spineCPGscaling;
    Matrix<double, 6, 1> trunk_kin;
    Matrix<double, 5, 3> FL_kin, FR_kin, HL_kin, HR_kin;
    Matrix<double, 2, 1> Duty;
    Matrix<double, 4, 1> phShifts;
    Matrix<double, 4, 1> stancePhase, swingPhase, legPhase;
    Matrix<double, 4, 1> legTrackingError;
    Matrix<double, 3, 4> GRF;
    //joystick manipulation
    double stick_r_lim, joy_walk_max_freq, joy_walk_min_freq, joy_walk_speed_change_filt, disable_crab_walk_lim;
    double posing_xFH, posing_yF, posing_zF, posing_head;
    double crab_rotAng, ellipse_small_axis;
    double posing_joy_x1_rate, posing_joy_y1_rate, posing_joy_y2_rate, posing_head_rate, posing_head_limit;
    double joy_swim_max_offset, joy_swim_max_freq, joy_swim_speed_change_filt, joy_swim_turning_dead_zone, aquastep_pitch_offset;

    //joystick
    double joy_x1, joy_x2, joy_y1, joy_y2, joy_x3, joy_y3;
    int joy_l1, joy_l2, joy_l3, joy_r1, joy_r2, joy_r3, joy_sel, joy_start, joy_bD, joy_bL, joy_bR, joy_bU;
    double joy_lsr, joy_rsr;
    double joy_lsphi, joy_rsphi;



    //reflexes 
    int stu_reflex_active[4], ext_reflex_active[4];
    double extRefForceLim;
    double extRefOnFilter;
    double extRefOffFilter;
    double extRefSpike;
    double extRefTimeout, stuRefTimeout;
    double stuRefForceLimX, stuRefForceLimZ;
    double stuRefOnFilter;
    double stuRefOffFilter;
    double stuRefDx;
    double stuRefDz;
    Matrix<double, 3, 3> fbck_fgirdRotMat, fgird2ForRotMat, forRotMat;


    double attData[3];
    Vector3d globalPosFL, globalPosFR, globalPosHL, globalPosHR;
    Vector3d gpsPos;
    double FoROrientation;
    Vector2d FFParam;

    // girdle trajectories
    Matrix<double, 6, 2> girdleTraj;
    Matrix<double, 3, 2> girdleVelocity, girdleVelocity_filtered;
    Matrix<double, 3, 2> forVelocity, forVelocity_filtered;
    Matrix<double, 1, 2> girdleAngularVelocity, girdleAngularVelocity_filtered;
    Matrix<double, 1, 2> forAngularVelocity, forAngularVelocity_filtered;
    Matrix<double, 6, 2> forTraj;
    Matrix<double, 3, 200> forTrajHist;
    Matrix<double, 4, 1> q0_trunk_from_spline;
    Matrix<double, 2, 2> girdleRealOrientation;
    Matrix<double, 2, 4> legStanceTraj;
    double walkingDirection, bodyAngularVelocity;
    double fgird_heading_angle;
    double t, dt;
    double walking_forward_velocity, walking_angular_velocity;

    // walking done properly
    Matrix<double, 3, 4> feetReference, feetFeedbackPosition;
    
    Vector2d girdleCpgOutput;
    double *joysticRecordings;
    int sizeOfJoystickRecording;

    double or_IGscaling, or_Ltot, or_Lf, or_Lh, or_Wf, or_Wh, or_Df, or_Dh, or_deltaLf, or_deltaLh, or_deltaWf, or_deltaWh,  or_deltaFH, or_ROLLvsYAW;
    Matrix<double, 2, 1>  or_spineCPGscaling;
    double or_heightF, or_heightH, or_lift_height_F, or_lift_height_H, or_stance_end_lift, or_stance_end_lift_exp_const;
    double or_phase_FL, or_deltaphi, or_walk_freq, legs_offset;

    Vector3d FgirdGpsPos;
    Matrix3d FgirdRotMat;
    Vector3d forRPY;
    Matrix<double, 4,4> CinvF, CinvH, MF, MH, or_MF, or_MH;
    Matrix<double, 3, 4> feetGPS;

    //================== public functions ===============================================
    Controller(double time_step); // constructor
    void setTimeStep(double time_step);
    bool runStep();
    void readJoystick();
    bool updateState();
    void getAngles(double table[MAX_NUM_MOTORS]);
    void getTorques(double table[MAX_NUM_MOTORS]);
    void forwardKinematics();
    std::vector<Matrix<double, 4, 4>>  legKinematics(Vector4d q, int leg);
    MatrixXd Jacob(Vector4d q, int leg);
    MatrixXd forceEstimation();
    void updateRobotState(double *d_posture, double *d_torque);
    void getForce(double *force);
    void getSensors(int acc, double *accData_i, int gyro, double *gyroData_i, int compass, double *compassData_i, int gps, double *gpsData_i);
    void getAttitude(double *rotmat);
    void globalKinematics(double gpsPosition[3], double rotMat[9]);
    void GRFfeedback(double grf[12]);

    

    
    
    // playing with torque
    bool DirectTorqueSetup();

    // estimation
    void getAcceleration(double acc[3]);
    void getFeetPosition(double *fl, double *fr, double *hl, double *hr);
    MatrixXd getReferenceFootsteps();
    void GetCompass(double data[3]);
    Vector3d getCoM();
    void torquePID();


    // static walking
    Vector3d getSwingTrajectory(Vector3d initPoint, Vector3d middlePoint, Vector3d finalPoint, double phase, int leg);
    bool moveSwingLeg(int leg);
    void moveBody(Vector3d bodyVelocity, double headingAngularVelocity);
    void moveGirdle(Vector3d girdleVelocity, double girdleAngularVelocity, int girdleNum);
    void walkingStateMachine();
    Vector3d trunkForwardKinematics(Vector3d fgird_pos, MatrixXd q_trunk);
    void getWalkingFrequency();

    //std::vector<Matrix<double,3,4>> predictedFootsteps;
    std::vector<Matrix<double,3,4>> supportPolys;
    MatrixXd followCoMReference(MatrixXd comref);
    void initOrobot();
    void setOrobotParameters(double or_freq, double or_spine, double or_height, double or_rvy);
    void parseParameterString(string str);
    void getGPS(double *gps, double *gps_feet1, double *gps_feet2, double *gps_feet3, double *gps_feet4);



  private:
    //================== private variables ===============================================
    vector<qpOASES::QProblemB> ikinQpSolver; 
    int useAnDF, useAnDH, useAnSP, ikin_maxIter;
    double ikin_tol, ikin_max_speed;
    double T_trans, T_trans0;
    Vector3d ikin_constr_penalty;
    // joystick objects
    js_event event;
    double maxSpeed;



    Matrix<double, 3, 4> midStance;
    Matrix<double, 1, 4> ellipse_a, ellipse_b, swing_height, swing_width;
    double trAnglesF0[4], trAnglesH0[4], bezierParamF0[2], bezierParamH0[2];
    Matrix<double, 3, 4> stanceStart, stanceEstEnd;

    // constraints on joint angles
    Matrix<double, 2, 4> constrFL, constrHL, constrFR, constrHR; 
    double constrS;


    Matrix<double, 3, 1> rFL, rFR, rHL, rHR, pFL, pFR, pHL, pHR, tmp3, rFL_posing, rFR_posing, rHL_posing, rHR_posing;
    Matrix<double, 4, 1> reCon, rsCon, vmCon;
	double z_offset;
    double angSignsCorrAnimal[MAX_NUM_MOTORS], angShiftsCorrAnimal[MAX_NUM_MOTORS];
    double angSignsCorrIkin2Webots[MAX_NUM_MOTORS], angShiftsCorrIkin2Webots[MAX_NUM_MOTORS];
    double angSignsCorrIkin2Robot[MAX_NUM_MOTORS], angShiftsCorrIkin2Robot[MAX_NUM_MOTORS];
    double angSignsCorrWebots2Robot[MAX_NUM_MOTORS], angShiftsCorrWebots2Robot[MAX_NUM_MOTORS];



    // animal data
    Matrix<double, N_anTr, 11> animalSpine;
    Matrix<double, N_anTr, 16> animalLegs;
    Matrix<double, Dynamic, 1> angles, torques;


    // initial conditions
    Vector4d q0FL, q0FR, q0HL, q0HR, qFL, qFR, qHL, qHR;
    Matrix<double, Dynamic, 1> init_angles;


    // auxiliary variables
    Matrix<double, Dynamic, 1> qs;
    
    double max_dist;
    double headq;
    Vector2d lamF, lamH;


    Vector2d posF, posH, posF2, posH2;
    double girdHiAng, R;
    // Vector3d pFL, pFR, pHL, pHR;
    Matrix2d rot2;
    Matrix3d rot3;

    // dynamics
    double spd; //spine
    double Tf1; // trajectory filtering
    double Tfilt_angles;

    // filter parameters
    Transform<double,3,Affine> HFL, HHL, HFR, HHR;

    double  tmps;
    double forceFilterExt, forceFilterStu;

    // trajectories
    //double nfl, nfr, nhl, nhr, rFLy, rFRy, rHLy, rHRy, fxfl, fxfr, fxhl, fxhr;
    



    //
    double animalAnglesWalking[1000][MAX_NUM_MOTORS], animalAnglesSwimming[1000][11], animalAnglesAquastepping[1000][MAX_NUM_MOTORS], animalAnglesNonperiodic[1000][MAX_NUM_MOTORS];
    int isTalking;

    // obstacles
    Matrix<double, 100, 3> gatePos, obsPos;
    Matrix<double, 3000, 2> path;
    int gateNum, obsNum, pathNum;
    double gateDist;
    double gateLen;



    double xGain, yGain, Troll_posture, Tpitch_posture;
    

    Matrix<double, Dynamic, 1> spine_gains0;
    Matrix<double, 4, 1> legs_height0;
    double erRefOnFilt, erRefOffFilt, er_timeout, er_duration, er_flow_limit;
    double er_slipping_Tfilt, er_slipping_threshold;


    //swimming
    Matrix<double, 16, 1> swim_legs_pos;

    //VMC
    double GW_spring_stiffness_z, GW_spring_damping_z, GW_spring_stiffness_xy, GW_spring_damping_xy, GW_Zref_initial;
    double GW_x_offset, GW_y_offset;
    double GW_Tfilt;

    double VMC_roll_ref_front, VMC_roll_ref_hind, VMC_roll_spring_stiffness_front, VMC_roll_spring_stiffness_hind, VMC_roll_damping_front, VMC_roll_damping_hind;

    // SNAKE STUFF
    double amp_wave_snake, freq_snake, snake_phase_lag, snake_adapt_gain, snake_adapt_filter;
    double snake_P_gain, snake_D_gain, snake_I_gain, snake_torque_filt, snake_torque_damping_filt;
    double snake_path_gain, snake_path_acceptance_region, snake_path_WP_number, snake_path_LAD, snake_path_limit;
    Matrix<double, 50, 2> snake_path_WP;
    Matrix<double, 12, 1> spine_forces;
    
    

    	
	
	
	// Animal data for Inverse Kinematics
    double animalSwing[1000][16], animalStance[1000][16];
    int animalSwingLen[4], animalStanceLen[4];
    
    
	   
	// trackways
    double steeringSpinelessGains[3];
    
	
	
	// akio stuff
    double akio_legs_sigma_v, akio_legs_sigma_h, akio_legs_sigma_mag;
	
	Matrix<double, 5, 1> spineGirdleCPGsWeights;
    Matrix<double, 2, 1> girdleOscAmp, girdleOscOffset, akio_spine_sigma;

    Matrix<double, 4, 23> masses;
    Matrix<double, 3, 1> CoM;
	


    double torque_D_filt, torque_error_filt, output_torque_filt, vmc_bodyPosture_stiffness, vmc_bodyPosture_damping;

    // force sensors and friction cones
    double coneForceFilter, coneFriction, forceTreshold;   
    Matrix<double, 4, 1> feet_friction_angles, feet_is_slipping;
    
    //Matrix<double, Dynamic, Dynamic> predLegPhase;
    
    //================== private functions ===============================================

    Vector4d iKinNullIDLS(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, Matrix4d M, 
                            double max_dist, int maxIter, double tol, MatrixXd constr, Vector3d constr_penalty);
    Vector4d iKinQpOases(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, Matrix4d M, 
                            double max_dist, int maxIter, double tol, MatrixXd constr, double max_speed, double* err);
    void joystickManipulation();
    void joystickRecord();
    void joystickReplay();
    bool getParameters();
    void walkingTrajectories();
    void legExtension();
    void stumbleReflex();
    void emergencyReflex();
    void swimFun();
    Vector3d contactDetection();
    void Kalman();
    void postureControl();
    MatrixXd transformation_Ikin_Webots(MatrixXd joint_angles, int direction, int shifts);
    MatrixXd transformation_Ikin_Robot(MatrixXd joint_angles, int direction, int shifts);
    MatrixXd transformation_Webots_Robot(MatrixXd joint_angles, int direction, int shifts);
    void swimFunCPG();
    void inverseKinematicsController();
    void slippingFromFrictionCone();
    MatrixXd CPGnetwork(Matrix<double, 22, 1> nu, Matrix<double, 22, 1> R, Matrix<double, 22, 1> a, Matrix<double, 22, 22> w, Matrix<double, 22, 22> ph_lag, Matrix<double, 22, 1> r0, Matrix<double, 22, 1> phi0, Matrix<double, 22, 1> fbck, double cpg_offset);
    void legPhaseDynamics(double dt);
    void girdleTrajectories(double v, double w);
    void legSwingTrajectory();
    void legStanceTrajectory();
    void getLegJacobians();
    void VMC_bodyPosture();
    std::vector<Matrix<double,3,4>> predictTrajectories(int N, double time_step, MatrixXd *predLegPhase);
    void girdleOscillations();
    void FixedGirdleOscillations();
    void getSwingStanceEndPoints();
    


    
};


#endif
