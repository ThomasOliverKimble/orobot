
#include "robotSim.hpp"


extern int USE_FEET, NUM_MOTORS, SKIP_WRIST_SERVOS;



pthread_t thread1;    
extern double get_timestamp();




/* RobotSim constructor */
RobotSim :: RobotSim(int TIME_STEP)
{


    gyro=new webots::Gyro("head_gyro");
    gyro->enable(TIME_STEP);
    acc=new webots::Accelerometer("head_acc");
    acc->enable(TIME_STEP);
    compass=new webots::Compass("compass_fgirdle");
    compass->enable(TIME_STEP);
    gps=new webots::GPS("gps_fgirdle");
    gps->enable(TIME_STEP);
    gyroData=gyro->getValues();
    accData=acc->getValues();
    compassData=compass->getValues();
    gpsData=gps->getValues();

    rec=new webots::Receiver("receiver");
    rec->enable(TIME_STEP);

    roboDef=getFromDef("OROBOT"); 
    
    roboPos=roboDef->getField("translation");
    roboRot=roboDef->getField("rotation");


    //webots::Field fldTranslation = roboDef.getField("translation");
    // INITIALIZE MARKERS
    
    FL_marker=getFromDef("foot_fl"); 
    FR_marker=getFromDef("foot_fr"); 
    HL_marker=getFromDef("foot_hl"); 
    HR_marker=getFromDef("foot_hr"); 
  
    CoM_marker=getFromDef("COM_MARKER"); 

    vector<string> RM_NAMES = 
    {
        "rm_s1","rm_s2","rm_s3","rm_s4","rm_s5","rm_s6","rm_s7","rm_s8",
        "rm_FLyaw", "rm_FLpitch", "rm_FLroll", "rm_FLelbow",
        "rm_FRyaw", "rm_FRpitch", "rm_FRroll", "rm_FRelbow",
        "rm_HLyaw", "rm_HLpitch", "rm_HLroll", "rm_HLknee",
        "rm_HRyaw", "rm_HRpitch", "rm_HRroll", "rm_HRknee",
        "rm_FLwrist", "rm_FRwrist", "rm_HLwrist", "rm_HRwrist"
    };

    vector<string> PS_NAMES = 
    {
        "ps_s1","ps_s2","ps_s3","ps_s4","ps_s5","ps_s6","ps_s7","ps_s8",
        "ps_FLyaw", "ps_FLpitch", "ps_FLroll", "ps_FLelbow",
        "ps_FRyaw", "ps_FRpitch", "ps_FRroll", "ps_FRelbow",
        "ps_HLyaw", "ps_HLpitch", "ps_HLroll", "ps_HLknee",
        "ps_HRyaw", "ps_HRpitch", "ps_HRroll", "ps_HRknee",
        "ps_FLwrist", "ps_FRwrist", "ps_HLwrist", "ps_HRwrist"
    };




    rm_motor.resize(NUM_MOTORS);
    ps_motor.resize(NUM_MOTORS);

    // get the motors
    cout << "connecting to motors" << endl;



    for(int i=0;i<(NUM_MOTORS);i++)
    {   
        rm_motor[i] = webots::Robot::getMotor(RM_NAMES[i]);
        ps_motor[i] = webots::Robot::getPositionSensor(PS_NAMES[i]);
        ps_motor[i]->enable(TIME_STEP);
        rm_motor[i]->enableTorqueFeedback(TIME_STEP);
    }
    cout << "motors collected" << endl;

   
    
    fgirdle=getFromDef("FGIRDLE"); 
    rotMat = fgirdle->getOrientation();
    
    

}

/* Sets the position of all servos to a table of angles in radians */
void
RobotSim :: getGRF(double *GRF)
{
    
    double finalForce[12];

    double ql = rec->getQueueLength();


    // set forces to zero
    for(int i=0; i<12; i++){
        GRF[i]  = 0;
    }

    // average the force over all physics time steps within a single controller time step
    while (rec->getQueueLength() > 0) {
        //cout << rec->getQueueLength() << endl;
        memcpy(finalForce, rec->getData(), sizeof(finalForce));
        //finalForce = (double*)rec->getData();
        rec->nextPacket();
        for(int i=0; i<12; i++){
            GRF[i]  += finalForce[i];
        }
    }
    // set forces to zero
    for(int i=0; i<12; i++){
        GRF[i]/=ql;
    }



}

/* Sets the position of all servos to a table of angles in radians */
void
RobotSim :: setAngles(double *table, int *ids)
{
    for(int i=0; i<(NUM_MOTORS);  i++){        
        if(ids[i]){
            rm_motor[i]->setPosition(table[i]);
        }
    }

}

/* Reads positions   */
void
RobotSim::getPositionFeedback(double *d_posture)
{
    for(int i=0;i<(NUM_MOTORS);i++)
    {        
        d_posture[i]=ps_motor[i]->getValue(); 
    }
}

/* Reads  torques   */
void
RobotSim::getTorqueFeedback(double *d_torques)
{
    for(int i=0;i<(NUM_MOTORS);i++)
    {        
        d_torques[i]=rm_motor[i]->getForceFeedback(); 
    }
}



/* Reads positions, torques and IMU data */
void
RobotSim::GetGPS(double *gpsData_i)
{
    gpsData_i[0]=gpsData[0];
    gpsData_i[1]=gpsData[1];
    gpsData_i[2]=gpsData[2];
}
/* Initializes IMU */
void
RobotSim::InitIMU()
{
    
    fgirdle=getFromDef("FGIRDLE"); 
    rotMat = fgirdle->getOrientation();
}


/* Reads positions, torques and IMU data */
void
RobotSim::ReadIMUAttitude(double *rotmat)
{
    

    double rotmat_tmp[9];
    rotMat = fgirdle->getOrientation();

    for(int i=0; i<9; i++){
        rotmat[i]=rotMat[i];
        rotmat_tmp[i]=rotMat[i];
    }
    rotmat[3]=rotmat_tmp[6];
    rotmat[4]=rotmat_tmp[7];
    rotmat[5]=rotmat_tmp[8];
    rotmat[6]=rotmat_tmp[3];
    rotmat[7]=rotmat_tmp[4];
    rotmat[8]=rotmat_tmp[5];

    for(int i=0; i<9; i++){
        rotmat_tmp[i]=rotmat[i];
    }

    rotmat[1]=rotmat_tmp[2];
    rotmat[4]=rotmat_tmp[5];
    rotmat[7]=rotmat_tmp[8];
    rotmat[2]=rotmat_tmp[1];
    rotmat[5]=rotmat_tmp[4];
    rotmat[8]=rotmat_tmp[7];

    rotmat[1]*=-1;
    rotmat[3]*=-1;
    rotmat[5]*=-1;
    rotmat[7]*=-1;

}




/* Quits simulation */
void
RobotSim::killSimulation()
{
    simulationQuit(0);
}



/* Sets position ond orientation of the Robot */
void 
RobotSim::setPositionRotation(double *p, double *r)
{
    roboPos->setSFVec3f((const double*)p);
    roboRot->setSFRotation(r);
}

/* Sets position of CoM marker */
void 
RobotSim::setCoMmarker(double *p)
{
    CoM_marker_pos->setSFVec3f((const double*)p);
}


/* Reads compass data */
void
RobotSim::GetCompass(double *data)
{
    data[0]=compassData[0];
    data[1]=compassData[1];
    data[2]=compassData[2];
}



/* Reads compass data */
void
RobotSim::GetFeetGPS(double *FL_feet_gpos, double *FR_feet_gpos, double *HL_feet_gpos, double *HR_feet_gpos)
{
    const double *FL_feet_gposC=FL_marker->getPosition();
    const double *FR_feet_gposC=FR_marker->getPosition();
    const double *HL_feet_gposC=HL_marker->getPosition();
    const double *HR_feet_gposC=HR_marker->getPosition();
   // cout<<FL_feet_gpos[0]<<FL_feet_gpos[1]<<FL_feet_gpos[2]<<endl;
    
    for(int i=0; i<3; i++){
        FL_feet_gpos[i]=FL_feet_gposC[i];
        FR_feet_gpos[i]=FR_feet_gposC[i];
        HL_feet_gpos[i]=HL_feet_gposC[i];
        HR_feet_gpos[i]=HR_feet_gposC[i];
    }
}







void
RobotSim::set2segFeetParam(double spring1, double spring2)
{   

    const char *feetDefs[4]={"foot_fl", "foot_fr", "foot_hl", "foot_hr"};
    webots::Node *feet_node[4];
    for(int i=0; i<4; i++){
        feet_node[i]=getFromDef(feetDefs[i]);
    }
    
    webots::Field *springConstant1, *springConstant2, *springConstant3;
    webots::Field *dampingConstant1, *dampingConstant2, *dampingConstant3;

    double spring3 = spring1/2;  //finger stiffness
    double damping1=0, damping2=0, damping3=0;

    // feet properties
    // FRONT
    double Im1[4], Im2[4], Im3[4];
    double W = 0.083;   //width
    double L1 = 0.057;  //length
    double L2 = 0.045;  //length fingers
    double D = 0.03;    //thicknes
    double m1 = 0.02;   //mass palm
    double m2 = 0.016;  // mass toes

    Im1[0] = m1/12*(D*D + L1*L1) + m2/12*(D*D + L2*L2) + m2*(L1/2 + L2/2)*(L1/2 + L2/2);
    Im2[0] = m1/12*(D*D + W*W) + m2/12*(D*D + W*W);
    Im3[0] = m2/12*(D*D + L2*L2) + m2*(L2/2)*(L2/2);

    Im1[1] = Im1[0];
    Im2[1] = Im2[0];
    Im3[1] = Im3[0];

    //HIND
    W = 0.091;   //width
    L1 = 0.061;  //length
    L2 = 0.051;  //length fingers
    D = 0.03;    //thicknes
    m1 = 0.024;   //mass palm
    m2 = 0.02;  // mass toes

    Im1[2] = m1/12*(D*D + L1*L1) + m2/12*(D*D + L2*L2) + m2*(L1/2 + L2/2)*(L1/2 + L2/2);
    Im2[2] = m1/12*(D*D + W*W) + m2/12*(D*D + W*W);
    Im3[2] = m2/12*(D*D + L2*L2) + m2*(L2/2)*(L2/2);

    Im1[3] = Im1[2];
    Im2[3] = Im2[2];
    Im3[3] = Im3[2];


    double eta = 2;

    for(int i=0; i<4; i++){
        springConstant1 = feet_node[i]->getField("springConstant1"); //wrist pitch
        springConstant2 = feet_node[i]->getField("springConstant2"); //wrist roll
        springConstant3 = feet_node[i]->getField("springConstant3"); //fingers
        springConstant1->setSFFloat(spring1);
        springConstant2->setSFFloat(spring2);
        springConstant3->setSFFloat(spring3);

        damping1 = 2*eta*sqrt(spring1*Im1[i]);
        damping2 = 2*eta*sqrt(spring2*Im1[i]);
        damping3 = 2*eta*sqrt(spring3*Im1[i]);





        dampingConstant1 = feet_node[i]->getField("dampingConstant1"); //wrist pitch
        dampingConstant2 = feet_node[i]->getField("dampingConstant2"); //wrist roll
        dampingConstant3 = feet_node[i]->getField("dampingConstant3"); //fingers
        dampingConstant1->setSFFloat(damping1);
        dampingConstant2->setSFFloat(damping2);
        dampingConstant3->setSFFloat(damping3);
        cout << "S1: " << spring1 << "\tS2: " << spring2 << "\tS3: " << spring3 << endl;
        cout << "D1: " << damping1 << "\tD2: " << damping2 << "\tD3: " << damping3 << endl;

    }

}


void
RobotSim::startVideoRecording(const string& filename, int width, int height)
{
    movieStartRecording(filename, width, height, 1, 100, 1, false);
}

void
RobotSim::stopVideoRecording()
{
    movieStopRecording();
}