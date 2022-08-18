#include "controller.hpp"
#include <iostream>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include <Eigen/Geometry>


using namespace std;
using namespace Eigen;



/* Controller constructor */
Controller :: Controller(double time_step) : HJs(12), HJs_g(12), HJfl(5), HJfr(5), HJhl(5), HJhr(5), HJfl_g(5), 
                                            HJfr_g(5), HJhl_g(5), HJhr_g(5), legJacob(4),ikinQpSolver(4)
{
    setTimeStep(time_step);


    //############################# RESIZE DYNAMIC MATRICES
    fbck_torques.resize(NUM_MOTORS,1);
    fbck_position.resize(NUM_MOTORS,1);
    joint_angles.resize(NUM_MOTORS,1);

    global_joint_pos.resize(3, NUM_MOTORS);

    angles.resize(NUM_MOTORS,1);
    torques.resize(NUM_MOTORS,1);


    init_angles.resize(NUM_MOTORS,1);
    spine_kin.resize(SPINE_SIZE,1);
    qs.resize(SPINE_SIZE,1);
    spine_gains0.resize(SPINE_SIZE,1);

    //############################ PARAMETERS ########################################################
    getParameters();

    cout<<"I successfully read the parameters"<<endl;
    state=INITIAL;


    t=0;
    if(USE_JOYSTICK){
        js.load(JOYSTICK_DEVNAME);

        if(js.is_ready()) {
            printf("Joystick Ready\n");
            updateState();
        }
    }


    headq=0;
    gamma=0;
    //########## STANDING POSITION #################

    force_filt << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    qs.setZero(SPINE_SIZE,1);

    // initial conditions
    qFL=q0FL;
    qFR=q0FR;
    qHL=q0HL;
    qHR=q0HR;

    init_angles.setZero(NUM_MOTORS,1);
    init_angles(10,0)= my_pi/2;
    init_angles(14,0)= my_pi/2;
    init_angles(19,0)= -my_pi/2;
    init_angles(23,0)= my_pi/2;



    cout << "NUM MOTORS:\t" << NUM_MOTORS << endl;
    cout << "SPINE_SIZE:\t" << SPINE_SIZE << endl;

    feetReference = midStance;



    legs_stance << 1,1,1,1;
    swingPhase << 0, 0, 0, 0;


    angles=init_angles;
    joint_angles=angles;


    feet_force<<0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    fbck_position=MatrixXd::Zero(NUM_MOTORS,1);
    freq_walk=0;



    girdleVelocity_filtered=MatrixXd::Zero(3,2);
    girdleAngularVelocity_filtered=MatrixXd::Zero(1,2);
    forVelocity_filtered=MatrixXd::Zero(3,2);
    forAngularVelocity_filtered=MatrixXd::Zero(1,2);

    //#############################Joystick dynamics########################################

    spd=0;

    isTalking=0;
    posing_xFH=0; posing_yF=0; posing_zF=0; posing_head=0;
    crab_rotAng=0;



    //############################# LEG PHASES ########################################
    for(int i=0; i<4; i++){
        legPhase(i)=phShifts(i)<0?phShifts(i)+1:phShifts(i);
    }
    
    girdleTraj << 0,0,0,0,0,0, 0,0,0,0,0,0;
    girdleTraj(3,0)=-IG; girdleTraj(3,1)=-IG;
    //girdleRealOrientation << 0,0, 0,0;
    forTraj << 0,0,0, 0,0,0, 0,0,0, 0,0,0;
    forTraj(3,0)=-IG; forTraj(3,1)=-IG;
    
    fgird_heading_angle=0;
    cout << "CONTROLLER CONSTRUCTOR DONE" << endl;
}

/* Sets time step */
void
Controller :: setTimeStep(double time_step)
{   
    dt=time_step;
    return;
}

/* Updates angles for all joints - MAIN FUNCTION */
bool
Controller :: runStep()
{



    if(!updateState()){
        return false;
    }
    
    //state=INITIAL;
    //joint_angles=angles;
    joint_angles=fbck_position;
    forwardKinematics();
    //static ofstream feetLogTest("feetLogTest.txt");
    //Matrix4d H=legKinematics(joint_angles.block<4,1>(23,0), 3);
    //feetLogTest << feetReference.block<3,1>(0,3).transpose() <<"\t" << H.block<3,1>(0,3).transpose() << endl;
    //feetLogTest << legTrackingError.transpose() << endl;
    //############################ TIME #########################################################
    if(state==WALKING|| state==SWIMMING || STANDING){
        t+=dt;
    }



    //--------------------------- joystick input-----------------------------
    if(state==WALKING){
        //joy_lsr=1;
        //walking_forward_velocity=or_Ltot*or_walk_freq*joy_lsr;
        //freq_walk=or_walk_freq*joy_lsr;
        gamma=pt1((joy_x2), gamma, 0.2, dt);
        walking_angular_velocity = (-gamma)*walking_forward_velocity*2;

        //-------------------------------------------------------------
        getSwingStanceEndPoints();

        walkingStateMachine();

    
    }
    
    //cout << feetReference << endl << endl;
    if(state==WALKING || state==STANDING || state==POSING){

        pFL=feetReference.block<3,1>(0,0);
        pFR=feetReference.block<3,1>(0,1);
        pHL=feetReference.block<3,1>(0,2);
        pHR=feetReference.block<3,1>(0,3);

        if(LOG_DATA){
            //feetReference.block<3,1>(0,leg) = AngleAxisd(forTraj(2+3*(leg/2),1)-girdleTraj(2+3*(leg/2),1), Vector3d::UnitZ())*trajPoint;
            static ofstream feetRefLog("feetRefLog.txt");
            Vector4d tmpv4;
            tmpv4.block<3,1>(0,0)=feetReference.block<3,1>(0,0);
            tmpv4(3,0)=1;
            tmpv4.block<3,1>(0,0) << AngleAxisd(-(forTraj(2+3*(0/2),1)-girdleTraj(2+3*(0/2),1)), Vector3d::UnitZ())*tmpv4.block<3,1>(0,0);
            //feetRefLog << (Fgird*tmpv4).transpose() << "\t";
            feetRefLog << tmpv4.transpose() << "\t";
            tmpv4.block<3,1>(0,0)=feetReference.block<3,1>(0,2);
            tmpv4(3,0)=1;
            tmpv4.block<3,1>(0,0) << AngleAxisd(-(forTraj(2+3*(2/2),1)-girdleTraj(2+3*(2/2),1)), Vector3d::UnitZ())*tmpv4.block<3,1>(0,0);
            //feetRefLog << (Hgird*tmpv4).transpose() << endl;
            tmpv4(0) -=-IG-Hgird(0, 3);
            feetRefLog << tmpv4.transpose() << endl;

        }

        inverseKinematicsController(); 
        angles.block(0,0,SPINE_SIZE,1)=qs;
        angles.block<4,1>(SPINE_SIZE+0 ,0)=qFL;
        angles.block<4,1>(SPINE_SIZE+4 ,0)=qFR;
        angles.block<4,1>(SPINE_SIZE+8 ,0)=qHL;
        angles.block<4,1>(SPINE_SIZE+12,0)=qHR;
        //angles=init_angles; //force initial posture
        joint_angles=angles;

        if(LOG_DATA){
            static ofstream anglesLog("anglesLog.txt");
            anglesLog << angles.transpose() << endl;
        }

    }


    return true;
}



/* Writes angles calculated by runStep function into a table - interface with Robot class */
void
Controller :: getAngles(double table[MAX_NUM_MOTORS])
{

        static MatrixXd angles_old=init_angles;
        static MatrixXd angles2=transformation_Ikin_Robot(init_angles,1,1);
        static MatrixXd angles2_old=transformation_Ikin_Robot(init_angles,1,1);
        

        angles=pt1_vec(angles, angles_old, T_trans, dt);

        // ============================== WRIST YAW COMPENSATION ======================================
        angles(NUM_MOTORS-4)=-atan2(HJfl_g[4](0,1), HJfl_g[4](1,1));
        angles(NUM_MOTORS-3)=-atan2(HJfr_g[4](0,1), HJfr_g[4](1,1));
        angles(NUM_MOTORS-2)= atan2(HJhl_g[4](1,0), HJhl_g[4](0,0));
        angles(NUM_MOTORS-1)=-atan2(HJhr_g[4](1,0), HJhr_g[4](0,0));

        /*angles(NUM_MOTORS-4)=(forTraj(2+3*(0/2),1)-girdleTraj(2+3*(0/2),1));
        angles(NUM_MOTORS-3)=(forTraj(2+3*(0/2),1)-girdleTraj(2+3*(0/2),1));
        angles(NUM_MOTORS-2)=(forTraj(2+3*(2/2),1)-girdleTraj(2+3*(2/2),1));
        angles(NUM_MOTORS-1)=(forTraj(2+3*(2/2),1)-girdleTraj(2+3*(2/2),1));*/

        angles_old=angles;


        if(IS_SIMULATION){
            angles2=transformation_Ikin_Webots(angles,1, 1);
        }

        if(!IS_SIMULATION){
            angles2=transformation_Ikin_Robot(angles,1,1);
        }

        angles2_old=angles2;

        for(int i=0; i<NUM_MOTORS; i++){
            table[i]=angles2(i);
        }


        // CONSTRAINTS
        //for(int i=0; i<11; i++){
        //    table[i]=table[i]>constrS?constrS:table[i];
        //    table[i]=table[i]<-constrS?-constrS:table[i];
        //}
        

    return;
}

/* Writes torques calculated by runStep function into a table - interface with Robot class */

void
Controller :: getTorques(double table[MAX_NUM_MOTORS])
{
        static MatrixXd torques2(NUM_MOTORS,1);
        //static MatrixXd torques(NUM_MOTORS,1);

        static MatrixXd torques2_old=MatrixXd::Zero(NUM_MOTORS, 1);
        static MatrixXd torques_old=MatrixXd::Zero(NUM_MOTORS, 1);




        if(IS_SIMULATION){
            torques2=transformation_Ikin_Webots(torques, 1, 0);
        }

        if(!IS_SIMULATION){
            torques2=transformation_Ikin_Robot(torques, 1, 0);
        }


        for(int i=0; i<NUM_MOTORS; i++){
            table[i]=torques2(i);
        }


        // CONSTRAINTS
    if(!IS_SIMULATION){
        for(int i=0; i<NUM_MOTORS; i++){
            table[i]=table[i]>7?7:table[i];
            table[i]=table[i]<-7?-7:table[i];
        }
    }

    return;
}

/* Update robot positions and fbck_torques */
void
Controller :: updateRobotState(double *d_posture, double *d_torque)
{
    for(int i=0; i<NUM_MOTORS; i++){
        fbck_position(i)=d_posture[i];
        fbck_torques(i)=d_torque[i];
    }

    if(IS_SIMULATION){
        fbck_position=transformation_Ikin_Webots(fbck_position, -1, 1);
        fbck_torques=transformation_Ikin_Webots(fbck_torques, -1, 0);
    }
    else{
        fbck_position=transformation_Ikin_Robot(fbck_position, -1, 1);
        fbck_torques=transformation_Ikin_Robot(fbck_torques, -1, 0);
    }
    //FKIN_posture();
}


/* Get acc data */
void
Controller :: getAcceleration(double acc[3])
{
    accData[0]=acc[0];
    accData[1]=acc[1];
    accData[2]=acc[2];
}


/* Get IMU data */
void
Controller :: getAttitude(double *rotmat)
{
    for(int i=0; i<3; i++){
        fbck_fgirdRotMat(0,i)=rotmat[i];
        fbck_fgirdRotMat(1,i)=rotmat[i+3];
        fbck_fgirdRotMat(2,i)=rotmat[i+6];
    }


    forRotMat = fgird2ForRotMat.inverse()*fbck_fgirdRotMat;

    forRPY = forRotMat.eulerAngles(2,1,0);



    forRPY(0)=    atan2( forRotMat(2,1),forRotMat(2,2));
    forRPY(1)=   atan2( -forRotMat(2,0),sqrt(forRotMat(2,1)*forRotMat(2,1)+forRotMat(2,2)*forRotMat(2,2)));
    forRPY(2)=     atan2( forRotMat(1,0),forRotMat(1,1));








    //static ofstream rotmatlog("rotmatlog.txt");
    //for(int i=0; i<3; i++){
    //    rotmatlog << forRPY(i) << "\t";
    //}
    //rotmatlog << endl;

}


/* Get gps */
void
Controller :: getGPS(double *gps, double *gps_feet1, double *gps_feet2, double *gps_feet3, double *gps_feet4)
{
    for(int i=0; i<3; i++){
        feetGPS(i,0)=gps_feet1[i];
        feetGPS(i,1)=gps_feet2[i];
        feetGPS(i,2)=gps_feet3[i];
        feetGPS(i,3)=gps_feet4[i];
        gpsPos(i)=gps[i];
    }

}

/*update GRF feedback */
void 
Controller :: GRFfeedback(double grf[12])
{
    for(int i=0; i<4; i++){
        GRF(0, i) = grf[3*i];
        GRF(1, i) = grf[3*i+1];
        GRF(2, i) = grf[3*i+2];
    }
    

    if(LOG_DATA){
        static ofstream grfLog("grfLog.txt");
        for(int i=0; i<4; i++){
            grfLog << GRF.block<3,1>(0,i).transpose() << "\t";
        }
        grfLog << endl;
    }



}

void
Controller :: initOrobot()
{
    

    // scaling
    or_Ltot     =   IG * or_IGscaling * or_Ltot    ;
    or_Wf       =   IG * or_IGscaling * or_Wf      ;
    or_Wh       =   IG * or_IGscaling * or_Wh      ;
    or_deltaFH  =   IG * or_IGscaling * or_deltaFH ;

    // stride lengths
    or_Lf=or_Ltot*or_Df;
    or_Lh=or_Ltot*or_Dh;


    // walking speed
    walking_forward_velocity=or_Ltot*or_walk_freq;
    freq_walk=or_walk_freq;
    // Duty
    Duty(0)=or_Df;
    Duty(1)=or_Dh;

    // inter leg foot step distance conservation
    //or_deltaLh = -or_deltaFH + or_deltaLf + IG + ((-2-2*or_deltaphi + or_Df - or_Dh)*or_Lf)/(2*or_Df);
    //or_deltaphi=1-fmod((1/2.)*(or_Df - or_Dh + (2*or_Df*(-or_deltaFH + or_deltaLf - or_deltaLh + IG))/or_Lf)+1,1);

    // get or_deltaphi from leg offsets
    //or_deltaphi= 1-fmod((1/2.)*( or_Df - or_Dh + (2*or_Df*(-or_deltaFH + or_deltaLf - or_deltaLh + IG))/or_Lf),1);

    // get leg offsets from or_deltaphi
    //double kin_off=or_Ltot-or_Ltot*or_deltaphi-IG+or_deltaFH;
    double kin_off=or_Ltot*(1-or_deltaphi)-IG+or_deltaFH;  //dkin = or_deltaFH - IG + or_Ltot*phi
    or_deltaLf=kin_off  + legs_offset;
    or_deltaLh=0        + legs_offset;
    cout << "or_deltaFH= " << or_deltaFH  << endl;
    cout << "or_deltaLf= " << or_deltaLf <<" \t or_deltaLh= " << or_deltaLh << endl;
    cout << "delta_phi= " << or_deltaphi  << endl;
    cout << "IG= " << IG  << endl;
    cout << "or_Lf= " << or_Lf  << endl;


    // Phase
    cout << "Duty  = " << Duty << endl;
    //cout << "deltaPhi = " << or_deltaphi << endl;

    phShifts(0)=or_phase_FL;
    phShifts(1)=phShifts(0)+0.5;
    phShifts(2)=phShifts(0)+or_deltaphi;
    phShifts(3)=phShifts(2)+0.5;

    //cout << "phShifts= " << phShifts << endl;


    for(int i=0; i<4; i++){
        phShifts(i)=fmod(phShifts(i),1);
    }


    joint_angles.setZero();



    //############################# LEG PHASES ########################################
    for(int i=0; i<4; i++){
        legPhase(i)=phShifts(i)<0?phShifts(i)+1:phShifts(i);
    }


    spineCPGscaling=or_spineCPGscaling;


    //############################# IKIN ########################################
    MF=or_MF;
    MH=or_MH;

    double or_ROLLvsYAW_basis=pow(10, or_ROLLvsYAW);


    MF(0,0)*=sqrt(or_ROLLvsYAW_basis);
    MH(0,0)*=sqrt(or_ROLLvsYAW_basis);
    MF(2,2)*=1/sqrt(or_ROLLvsYAW_basis);
    MH(2,2)*=1/sqrt(or_ROLLvsYAW_basis);




    // new gait parameters
    ellipse_a(0)=or_Lf/2;
    ellipse_a(1)=or_Lf/2;
    ellipse_a(2)=or_Lh/2;
    ellipse_a(3)=or_Lh/2;



    midStance(0,0)=or_deltaLf;
    midStance(1,0)=or_Wf/2;
    midStance(2,0)=or_heightF;
    midStance.block<3,1>(0,1)=midStance.block<3,1>(0,0); midStance(1,1)*=-1;




    midStance(0,2)=or_deltaLh;
    midStance(1,2)=or_Wh/2;
    midStance(2,2)=or_heightH;

    midStance.block<3,1>(0,3)=midStance.block<3,1>(0,2); midStance(1,3)*=-1;


    swing_height(0)=or_lift_height_F;
    swing_height(1)=or_lift_height_F;
    swing_height(2)=or_lift_height_H;
    swing_height(3)=or_lift_height_H;

    swing_width(0)=or_deltaWf;
    swing_width(1)=-or_deltaWf;
    swing_width(2)=or_deltaWh;
    swing_width(3)=-or_deltaWh;

    cout << "ellipse_a= " << ellipse_a.transpose()  << endl;
    cout << "or_deltaLf= " << or_deltaLf <<" \t or_deltaLh= " << or_deltaLh << endl;
    cout << "delta_phi= " << or_deltaphi  << endl;

}

void
Controller :: setOrobotParameters(double or_freq, double or_spine, double or_height, double or_rvy)
{
    // freq
    walking_forward_velocity=or_Ltot*or_freq;
    freq_walk=or_freq;

    // spine
    spineCPGscaling(0)=or_spine;
    spineCPGscaling(1)=or_spine;

    // height
    midStance(0,0)=or_deltaLf;
    midStance(1,0)=or_Wf/2;
    midStance(2,0)=or_height;
    midStance.block<3,1>(0,1)=midStance.block<3,1>(0,0); midStance(1,1)*=-1;
    midStance(0,2)=or_deltaLh;
    midStance(1,2)=or_Wh/2;
    midStance(2,2)=or_height;
    midStance.block<3,1>(0,3)=midStance.block<3,1>(0,2); midStance(1,3)*=-1;

    // roll vs yaw
    MF=or_MF;
    MH=or_MH;

    double or_ROLLvsYAW_basis=pow(10, or_rvy);
    MF(0,0)*=sqrt(or_ROLLvsYAW_basis);
    MH(0,0)*=sqrt(or_ROLLvsYAW_basis);
    MF(2,2)*=1/sqrt(or_ROLLvsYAW_basis);
    MH(2,2)*=1/sqrt(or_ROLLvsYAW_basis);
}

void
Controller :: parseParameterString(string str)
{   
    static double or_freq=0.5, or_spine=0.4, or_height=-0.1914, or_rvy=0.4286;
    size_t found; 
    found = str.find("scaling");
    if (found!=string::npos){
        found = str.find("=");
        or_spine = stod(str.substr(found+1));
    }

    found = str.find("height");
    if (found!=string::npos){
        found = str.find("=");
        or_height = stod(str.substr(found+1));
    }

    found = str.find("roll");
    if (found!=string::npos){
        found = str.find("=");
        or_rvy = stod(str.substr(found+1));
    }

    found = str.find("frequency");
    if (found!=string::npos){
        found = str.find("=");
        or_freq = stod(str.substr(found+1));
    }

    setOrobotParameters( or_freq,  or_spine,  or_height,  or_rvy);







}