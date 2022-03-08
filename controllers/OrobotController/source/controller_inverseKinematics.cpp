#include "controller.hpp"

using namespace std;
using namespace Eigen;


/* Calculates forward kinematics for a single leg */
vector<Matrix4d>
Controller :: legKinematics(Vector4d q, int leg)
{

    static vector<Matrix4d> H(5);

    if(leg==0){     //FL
        H[0] <<  cos(q(0)), -sin(q(0)), 0, FL_kin(0,0),
                sin(q(0)), cos(q(0)), 0, FL_kin(0,1),
                0, 0, 1, FL_kin(0,2),
                0, 0, 0, 1;
        H[1] <<  1, 0, 0, FL_kin(1,0),
                0, cos(q(1)), -sin(q(1)), FL_kin(1,1),
                0, sin(q(1)), cos(q(1)), FL_kin(1,2),
                0, 0, 0, 1;
        H[2] <<  cos(q(2)), 0, sin(q(2)), FL_kin(2,0),
                0, 1, 0, FL_kin(2,1),
                -sin(q(2)), 0, cos(q(2)), FL_kin(2,2),
                0, 0, 0, 1;
        H[3] <<  cos(q(3)), -sin(q(3)), 0, FL_kin(3,0),
                sin(q(3)), cos(q(3)), 0, FL_kin(3,1),
                0, 0, 1, FL_kin(3,2),
                0, 0, 0, 1;
        H[4] <<  1, 0, 0, FL_kin(4,0),
                0, 1, 0, FL_kin(4,1),
                0, 0, 1, FL_kin(4,2),
                0, 0, 0, 1;
    }
    else if (leg==1){       //FR
        H[0] <<  cos(q(0)), -sin(q(0)), 0, FR_kin(0,0),
                    sin(q(0)), cos(q(0)), 0, FR_kin(0,1),
                    0, 0, 1, FR_kin(0,2),
                    0, 0, 0, 1;
        H[1] <<  1, 0, 0, FR_kin(1,0),
                    0, cos(q(1)), -sin(q(1)), FR_kin(1,1),
                    0, sin(q(1)), cos(q(1)), FR_kin(1,2),
                    0, 0, 0, 1;
        H[2] <<  cos(q(2)), 0, sin(q(2)), FR_kin(2,0),
                    0, 1, 0, FR_kin(2,1),
                    -sin(q(2)), 0, cos(q(2)), FR_kin(2,2),
                    0, 0, 0, 1;
        H[3] <<  cos(q(3)), -sin(q(3)), 0, FR_kin(3,0),
                    sin(q(3)), cos(q(3)), 0, FR_kin(3,1),
                    0, 0, 1, FR_kin(3,2),
                    0, 0, 0, 1;
        H[4] <<  1, 0, 0, FR_kin(4,0),
                    0, 1, 0, FR_kin(4,1),
                    0, 0, 1, FR_kin(4,2),
                    0, 0, 0, 1;
    }
    else if (leg==2){       //HL
        H[0] <<  cos(q(0)), -sin(q(0)), 0, HL_kin(0,0),
                    sin(q(0)), cos(q(0)), 0, HL_kin(0,1),
                    0, 0, 1, HL_kin(0,2),
                    0, 0, 0, 1;
        H[1] <<  1, 0, 0, HL_kin(1,0),
                    0, cos(q(1)), -sin(q(1)), HL_kin(1,1),
                    0, sin(q(1)), cos(q(1)), HL_kin(1,2),
                    0, 0, 0, 1;
        H[2] <<  cos(q(2)), 0, sin(q(2)), HL_kin(2,0),
                    0, 1, 0, HL_kin(2,1),
                    -sin(q(2)), 0, cos(q(2)), HL_kin(2,2),
                    0, 0, 0, 1;
        H[3] <<  1, 0, 0, HL_kin(3,0),
                    0, cos(q(3)), -sin(q(3)), HL_kin(3,1),
                    0, sin(q(3)), cos(q(3)), HL_kin(3,2),
                    0, 0, 0, 1;
        H[4] <<  1, 0, 0, HL_kin(4,0),
                    0, 1, 0, HL_kin(4,1),
                    0, 0, 1, HL_kin(4,2),
                    0, 0, 0, 1;
    }
    else if (leg==3){       //HR
        H[0] <<  cos(q(0)), -sin(q(0)), 0, HR_kin(0,0),
                    sin(q(0)), cos(q(0)), 0, HR_kin(0,1),
                    0, 0, 1, HR_kin(0,2),
                    0, 0, 0, 1;
        H[1] <<  1, 0, 0, HR_kin(1,0),
                    0, cos(q(1)), -sin(q(1)), HR_kin(1,1),
                    0, sin(q(1)), cos(q(1)), HR_kin(1,2),
                    0, 0, 0, 1;
        H[2] <<  cos(q(2)), 0, sin(q(2)), HR_kin(2,0),
                    0, 1, 0, HR_kin(2,1),
                    -sin(q(2)), 0, cos(q(2)), HR_kin(2,2),
                    0, 0, 0, 1;
        H[3] <<  1, 0, 0, HR_kin(3,0),
                    0, cos(q(3)), -sin(q(3)), HR_kin(3,1),
                    0, sin(q(3)), cos(q(3)), HR_kin(3,2),
                    0, 0, 0, 1;
        H[4] <<  1, 0, 0, HR_kin(4,0),
                    0, 1, 0, HR_kin(4,1),
                    0, 0, 1, HR_kin(4,2),
                    0, 0, 0, 1;
    }


    //return H[0]*H[1]*H[2]*H[3]*H[4];
    return H;
}

/* Returns jacobian matrix of one leg */
MatrixXd
Controller :: Jacob(Vector4d q, int leg)
{
    static MatrixXd J(3, 4);
    static double S1, S2, S3, S4, C1, C2, C3, C4;
    S1 = sin(q(0));
    S2 = sin(q(1));
    S3 = sin(q(2));
    S4 = sin(q(3));
    C1 = cos(q(0));
    C2 = cos(q(1));
    C3 = cos(q(2));
    C4 = cos(q(3));

    if(leg==0){     //FL
    J(0) = - 0.118*C1*C2 - 0.12*C4*(C3*S1 + C1*S2*S3) - 0.12*C1*C2*S4;
    J(1) = 0.12*C4*(C1*C3 - 1.0*S1*S2*S3) - 0.118*C2*S1 - 0.12*C2*S1*S4;
    J(2) = 0;
    J(3) = 0.002*S1*(59.0*S2 + 60.0*S2*S4 - 60.0*C2*C4*S3);
    J(4) = -0.002*C1*(59.0*S2 + 60.0*S2*S4 - 60.0*C2*C4*S3);
    J(5) = 0.118*C2 + 0.12*C2*S4 + 0.12*C4*S2*S3;
    J(6) = -0.12*C4*(C1*S3 + C3*S1*S2);
    J(7) = -0.12*C4*(S1*S3 - 1.0*C1*C3*S2);
    J(8) = -0.12*C2*C3*C4;
    J(9) = - 0.12*S4*(C1*C3 - 1.0*S1*S2*S3) - 0.12*C2*C4*S1;
    J(10) = 0.12*C1*C2*C4 - 0.12*S4*(C3*S1 + C1*S2*S3);
    J(11) = 0.12*C4*S2 + 0.12*C2*S3*S4;
        }
        else if (leg==1){       //FR
    J(0) = 0.118*C1*C2 - 0.12*C4*(C3*S1 + C1*S2*S3) - 0.12*C1*C2*S4;
    J(1) = 0.118*C2*S1 + 0.12*C4*(C1*C3 - 1.0*S1*S2*S3) - 0.12*C2*S1*S4;
    J(2) = 0;
    J(3) = -0.002*S1*(59.0*S2 - 60.0*S2*S4 + 60.0*C2*C4*S3);
    J(4) = 0.002*C1*(59.0*S2 - 60.0*S2*S4 + 60.0*C2*C4*S3);
    J(5) = 0.12*C2*S4 - 0.118*C2 + 0.12*C4*S2*S3;
    J(6) = -0.12*C4*(C1*S3 + C3*S1*S2);
    J(7) = -0.12*C4*(S1*S3 - 1.0*C1*C3*S2);
    J(8) = -0.12*C2*C3*C4;
    J(9) = - 0.12*S4*(C1*C3 - 1.0*S1*S2*S3) - 0.12*C2*C4*S1;
    J(10) = 0.12*C1*C2*C4 - 0.12*S4*(C3*S1 + C1*S2*S3);
    J(11) = 0.12*C4*S2 + 0.12*C2*S3*S4;
        }
        else if (leg==2){       //HL
    J(0) = - 0.128*C1*C2 - 0.12*S4*(S1*S3 - 1.0*C1*C3*S2) - 0.12*C1*C2*C4;
    J(1) = 0.12*S4*(C1*S3 + C3*S1*S2) - 0.128*C2*S1 - 0.12*C2*C4*S1;
    J(2) = 0;
    J(3) = 0.008*S1*(16.0*S2 + 15.0*C4*S2 + 15.0*C2*C3*S4);
    J(4) = -0.008*C1*(16.0*S2 + 15.0*C4*S2 + 15.0*C2*C3*S4);
    J(5) = 0.128*C2 + 0.12*C2*C4 - 0.12*C3*S2*S4;
    J(6) = 0.12*S4*(C1*C3 - 1.0*S1*S2*S3);
    J(7) = 0.12*S4*(C3*S1 + C1*S2*S3);
    J(8) = -0.12*C2*S3*S4;
    J(9) = 0.12*C4*(C1*S3 + C3*S1*S2) + 0.12*C2*S1*S4;
    J(10) = 0.12*C4*(S1*S3 - 1.0*C1*C3*S2) - 0.12*C1*C2*S4;
    J(11) = 0.12*C2*C3*C4 - 0.12*S2*S4;
        }
        else if (leg==3){       //HR
    J(0) = 0.128*C1*C2 + 0.12*S4*(S1*S3 - 1.0*C1*C3*S2) + 0.12*C1*C2*C4;
    J(1) = 0.128*C2*S1 - 0.12*S4*(C1*S3 + C3*S1*S2) + 0.12*C2*C4*S1;
    J(2) = 0;
    J(3) = -0.008*S1*(16.0*S2 + 15.0*C4*S2 + 15.0*C2*C3*S4);
    J(4) = 0.008*C1*(16.0*S2 + 15.0*C4*S2 + 15.0*C2*C3*S4);
    J(5) = 0.12*C3*S2*S4 - 0.12*C2*C4 - 0.128*C2;
    J(6) = -0.12*S4*(C1*C3 - 1.0*S1*S2*S3);
    J(7) = -0.12*S4*(C3*S1 + C1*S2*S3);
    J(8) = 0.12*C2*S3*S4;
    J(9) = - 0.12*C4*(C1*S3 + C3*S1*S2) - 0.12*C2*S1*S4;
    J(10) = 0.12*C1*C2*S4 - 0.12*C4*(S1*S3 - 1.0*C1*C3*S2);
    J(11) = 0.12*S2*S4 - 0.12*C2*C3*C4;
        }  
        

        return J;
}

/* Gets entire forward kinematics */
void 
Controller :: forwardKinematics()
{

    static vector<Matrix4d> HJs_g0(SPINE_SIZE+1), HJs0(SPINE_SIZE+1);

    


    // SPINE
    HJs[0]<<1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;
    HJs_g[0]=HJs[0];  



        


    for(int i=1; i<(SPINE_SIZE+1); i++){
        HJs[i]<<cos(joint_angles(i)), -sin(joint_angles(i)), 0, -spine_kin(i-1),
                sin(joint_angles(i)), cos(joint_angles(i)), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        HJs_g[i]=HJs_g[i-1]*HJs[i];

    }


    




    Fgird.block<3,3>(0, 0)=HJs_g[1].block<3,3>(0, 0);
    Hgird.block<3,3>(0, 0)=HJs_g[5].block<3,3>(0, 0);
    //Fgird.block<3,1>(0, 3) = 0.744329426092252*HJs_g[1].block<3,1>(0, 3) + 0.255670573907749*HJs_g[2].block<3,1>(0, 3);
    //Hgird.block<3,1>(0, 3) = 0.674934524726544*HJs_g[5].block<3,1>(0, 3) + 0.325065475273456*HJs_g[6].block<3,1>(0, 3);
    Fgird.block<3,1>(0, 3) = (spine_kin(1)-girdLoc(0))/spine_kin(1) * HJs_g[1].block<3,1>(0, 3) + (girdLoc(0))/spine_kin(1)*HJs_g[2].block<3,1>(0, 3);
    Hgird.block<3,1>(0, 3) = (spine_kin(5)-girdLoc(1))/spine_kin(5) * HJs_g[5].block<3,1>(0, 3) + (girdLoc(1))/spine_kin(5)*HJs_g[6].block<3,1>(0, 3);
    Fgird.block<3,3>(0, 0)=HJs_g[1].block<3,3>(0, 0);
    Hgird.block<3,3>(0, 0)=HJs_g[5].block<3,3>(0, 0);
    Fgird.block<1,4>(3, 0) << 0, 0, 0, 1;
    Hgird.block<1,4>(3, 0) << 0, 0, 0, 1;



    HJfl=legKinematics(joint_angles.block<4,1>(SPINE_SIZE   ,0), 0);
    HJfr=legKinematics(joint_angles.block<4,1>(SPINE_SIZE+4 ,0), 1);
    HJhl=legKinematics(joint_angles.block<4,1>(SPINE_SIZE+8 ,0), 2);
    HJhr=legKinematics(joint_angles.block<4,1>(SPINE_SIZE+12,0), 3);

    // FRONT LEFT LEG
    HJfl_g[0]=Fgird*HJfl[0];
    for(int i=1;i<5;i++){
        HJfl_g[i]=HJfl_g[i-1]*HJfl[i];
    }
    // FRONT RIGHT LEG
    HJfr_g[0]=Fgird*HJfr[0];
    for(int i=1;i<5;i++){
        HJfr_g[i]=HJfr_g[i-1]*HJfr[i];
    }

    // HIND LEFT LEG
    HJhl_g[0]=Hgird*HJhl[0];
    for(int i=1;i<5;i++){
        HJhl_g[i]=HJhl_g[i-1]*HJhl[i];
    }

    // HIND RIGHT LEG
    HJhr_g[0]=Hgird*HJhr[0];
    for(int i=1;i<5;i++){
        HJhr_g[i]=HJhr_g[i-1]*HJhr[i];
    }

    // TRANSFORM EVERYTHING INTO ROBOT's COO FRAME (passing through both girdles)
    static Matrix4d HTransform, HTranslate, HRotate;
   

    hgirdlePhi = atan2(Hgird(1, 3)-Fgird(1, 3), -Hgird(0, 3)+Fgird(0, 3));
    headq=hgirdlePhi;
    HTransform.block(0,0,3,3) << cos(hgirdlePhi), -sin(hgirdlePhi), 0,    sin(hgirdlePhi), cos(hgirdlePhi), 0,    0, 0, 1;
    HTransform.block(0,3,3,1) = -Fgird.block<3,1>(0,3);
    HTransform.block(3,0,1,4) << 0, 0, 0, 1;

    HTranslate=Matrix4d::Identity();
    HTranslate.block<3,1>(0,3) = -Fgird.block<3,1>(0,3);
    HRotate=Matrix4d::Identity();
    HRotate.block<3,3>(0,0) << cos(hgirdlePhi), -sin(hgirdlePhi), 0,    sin(hgirdlePhi), cos(hgirdlePhi), 0,    0, 0, 1;


    HTransform=HTransform.inverse().eval();
    HTransform=HRotate*HTranslate;
    for(int i=0;i<5;i++){
        HJfl_g[i]=HTransform*HJfl_g[i];
        HJfr_g[i]=HTransform*HJfr_g[i];
        HJhl_g[i]=HTransform*HJhl_g[i];
        HJhr_g[i]=HTransform*HJhr_g[i];
    }


    for(int i=0;i<SPINE_SIZE;i++){
        HJs_g[i]=HTransform*HJs_g[i];
    }

    Hgird=HTransform*Hgird;
    Fgird=HTransform*Fgird;

    // HEAD
    HJh<<cos(-headq+my_pi), -sin(-headq+my_pi), 0, 0,
                sin(-headq+my_pi), cos(-headq+my_pi), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;


    HJh_g=HJs_g[0]*HJh;



    // get fgird rotation in respect to the frame of reference
    fgird2ForRotMat=AngleAxisd(hgirdlePhi, Vector3d::UnitZ());


    FoROrientation=hgirdlePhi;








}


/* Gets trunk forward kinematics - returns position of Hind girdle for the given position of Front girdle and spine angles*/
Vector3d
Controller :: trunkForwardKinematics(Vector3d fgird_pos, MatrixXd q_trunk)
{

    MatrixXd spine_joints(3,4);
    //first joint
    spine_joints(0,0)=fgird_pos(0) + cos(fgird_pos(2))*(-trunk_kin(0));
    spine_joints(1,0)=fgird_pos(1) + sin(fgird_pos(2))*(-trunk_kin(0));
    spine_joints(2,0)=fgird_pos(2);

    //spine
    for(int i=1; i<4; i++){
        spine_joints(2,i)=q_trunk(i-1)+spine_joints(2,i-1);
        spine_joints(0,i)=spine_joints(0,i-1) + cos( spine_joints(2,i) ) * (-trunk_kin(i));
        spine_joints(1,i)=spine_joints(1,i-1) + sin( spine_joints(2,i) ) * (-trunk_kin(i));
    }

    //hind girdle
    Vector3d hgird_pos;
    hgird_pos(2)=spine_joints(2,3) + q_trunk(3);
    hgird_pos(0)=spine_joints(0,3) + cos( hgird_pos(2) ) * (-trunk_kin(4));
    hgird_pos(1)=spine_joints(1,3) + sin( hgird_pos(2) ) * (-trunk_kin(4));

    return hgird_pos;

}


/* Calculates global position of all the joints for a given GPS position and front girdle orientation */
void 
Controller :: globalKinematics(double gpsPosition[3], double rotMat[9])
{
    

    FgirdGpsPos(0)=gpsPosition[0];
    FgirdGpsPos(1)=-gpsPosition[2];
    FgirdGpsPos(2)=gpsPosition[1];
    Matrix<double,3,3,RowMajor> rotMatEig;

    for(int i=0; i<9; i++){
        rotMatEig(i)=rotMat[i];
    }



    FgirdRotMat = rotMatEig*Fgird.block<3,3>(0,0).inverse();    

    //Map<Matrix<double,3,3,RowMajor>> rotMatEig(rotMat)


    

    for(int i=0; i<SPINE_SIZE; i++){
        global_joint_pos.block<3,1>(0,i) =FgirdRotMat*HJs_g[i].block<3,1>(0, 3) + FgirdGpsPos;
    }
    for(int i=0; i<4; i++){
        global_joint_pos.block<3,1>(0,SPINE_SIZE   +i) = FgirdRotMat*HJfl_g[i].block<3,1>(0, 3) + FgirdGpsPos;
        global_joint_pos.block<3,1>(0,SPINE_SIZE+4 +i) = FgirdRotMat*HJfr_g[i].block<3,1>(0, 3) + FgirdGpsPos;
        global_joint_pos.block<3,1>(0,SPINE_SIZE+8 +i) = FgirdRotMat*HJhl_g[i].block<3,1>(0, 3) + FgirdGpsPos;
        global_joint_pos.block<3,1>(0,SPINE_SIZE+12+i) = FgirdRotMat*HJhr_g[i].block<3,1>(0, 3) + FgirdGpsPos;
    }
    global_feet_pos.block<3,1>(0,0) = FgirdRotMat*HJfl_g[4].block<3,1>(0, 3) + FgirdGpsPos;
    global_feet_pos.block<3,1>(0,1) = FgirdRotMat*HJfr_g[4].block<3,1>(0, 3) + FgirdGpsPos;
    global_feet_pos.block<3,1>(0,2) = FgirdRotMat*HJhl_g[4].block<3,1>(0, 3) + FgirdGpsPos;
    global_feet_pos.block<3,1>(0,3) = FgirdRotMat*HJhr_g[4].block<3,1>(0, 3) + FgirdGpsPos;



    //FoRroll=    atan2( FgirdRotMat(2,1),FgirdRotMat(2,2));
    //FoRpitch=   atan2( -FgirdRotMat(2,0),sqrt(FgirdRotMat(2,1)*FgirdRotMat(2,1)+FgirdRotMat(2,2)*FgirdRotMat(2,2)));
    //FoRyaw=     atan2( FgirdRotMat(1,0),FgirdRotMat(1,1));

    //cout << FoRroll*180/pi << "\t" << FoRpitch*180/pi <<"\t" <<FoRyaw*180/pi << endl;

} 

/* get global feet position from webots */
void
Controller :: getFeetPosition(double *fl, double *fr, double *hl, double *hr)
{
    globalPosFL(0)=fl[0];
    globalPosFL(1)=-fl[2];
    globalPosFL(2)=fl[1];

    globalPosFR(0)=fr[0];
    globalPosFR(1)=-fr[2];
    globalPosFR(2)=fr[1];

    globalPosHL(0)=hl[0];
    globalPosHL(1)=-hl[2];
    globalPosHL(2)=hl[1];

    globalPosHR(0)=hr[0];
    globalPosHR(1)=-hr[2];
    globalPosHR(2)=hr[1];

}

/* Calculates angles from IKIN and compensates for spine_kin */
void
Controller :: inverseKinematicsController()
{


    //############################ USE ANIMAL DATA AS PREFERED JOINT ANGLES #######################
    if(useAnDF){
        for(int i=0; i<4; i++){
            if(legs_stance(0))
                q0FL(i)=animalStance[(int)floor(stancePhase(0)*animalStanceLen[0])][i];     
            else
                q0FL(i)=animalSwing[(int)floor(swingPhase(0)*animalSwingLen[0])][i]; 


            if(legs_stance(1))
                q0FR(i)=animalStance[(int)floor(stancePhase(1)*animalStanceLen[1])][i+4];     
            else
                q0FR(i)=animalSwing[(int)floor(swingPhase(1)*animalSwingLen[1])][i+4]; 
        }
    }

    if(useAnDH){
        for(int i=0; i<4; i++){
            if(legs_stance(2))
                q0HL(i)=animalStance[(int)floor(stancePhase(2)*animalStanceLen[2])][i+8];     
            else
                q0HL(i)=animalSwing[(int)floor(swingPhase(2)*animalSwingLen[2])][i+8]; 


            if(legs_stance(3))
                q0HR(i)=animalStance[(int)floor(stancePhase(3)*animalStanceLen[3])][i+12];     
            else
                q0HR(i)=animalSwing[(int)floor(swingPhase(3)*animalSwingLen[3])][i+12]; 
        }
    }

    //############################ IKIN ###########################################################




    //double t1=get_timestamp();
    qFL=iKinQpOases(0, pFL, q0FL, qFL, lamF, MF, max_dist, ikin_maxIter, ikin_tol, constrFL, ikin_max_speed, &legTrackingError(0));
    qFR=iKinQpOases(1, pFR, q0FR, qFR, lamF, MF, max_dist, ikin_maxIter, ikin_tol, constrFR, ikin_max_speed, &legTrackingError(1));
    qHL=iKinQpOases(2, pHL, q0HL, qHL, lamH, MH, max_dist, ikin_maxIter, ikin_tol, constrHL, ikin_max_speed, &legTrackingError(2));
    qHR=iKinQpOases(3, pHR, q0HR, qHR, lamH, MH, max_dist, ikin_maxIter, ikin_tol, constrHR, ikin_max_speed, &legTrackingError(3));
    //cout << (get_timestamp()-t1)*1000 << endl;



}




/* Solves inverse kinematics for one leg by using Damped Inverse Jacobian method */
Vector4d
Controller :: iKinQpOases(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, Matrix4d M, 
                            double max_dist, int maxIter, double tol, MatrixXd constr, double max_speed, double* err)
{
    static MatrixXd J(3,4), H(4,4), D(4,4), U(3,3), V(4,4), S(3,1), Jnull(4,4);
    static MatrixXd H_qp(4,4), h_qp(4,1), lb(4,1), ub(4,1);
    static qpOASES::real_t H_qpoases[4*4], h_qpoases[4], lb_qpoases[4], ub_qpoases[4];
    static Vector4d dqref, dq;
    static Vector3d dpref, p0;
    static double norm_dp;
    static vector<bool> is_init={false, false, false, false}, is_init_null={false, false, false, false};
    vector<Matrix4d> H_vec(5);    

    // get current position
    H_vec=legKinematics(q0, leg);
    H=H_vec[0]*H_vec[1]*H_vec[2]*H_vec[3]*H_vec[4];
    p0=H.block<3, 1>(0, 3);
    // get reference velocities
    dpref=pref-p0;
    int k=0;
    // START ITERATIVE IKIN SOLVING
    for(k=0; k<maxIter; k++){
        //============================================ PRIMARY PROBLEM =================================================
        
        // get Jacobians
        J=Jacob(q0, leg);
        JacobiSVD<MatrixXd> svd(J, ComputeFullU | ComputeFullV);
        S=svd.singularValues();
        U=svd.matrixU();
        V=svd.matrixV();
        D=Matrix4d::Zero();
        D(3,3)=1;
        for(int i=1; i<0;i++){
            if(abs(S(i))<0.001)
                D(i, i)=1;
        }

        Jnull=V*D*V.transpose();
        // limit maximum linear velocity
        norm_dp=sqrt(dpref(0)*dpref(0)+dpref(1)*dpref(1)+dpref(2)*dpref(2));
        if(norm_dp>max_dist){
            dpref=max_dist/norm_dp*dpref;
        }
        // get reference velocities
        dqref=qref-q0;
        // construct qp matrices
        H_qp=J.transpose()*J + lam(0)*MatrixXd::Identity(4,4) + Jnull.transpose()*M*Jnull;
        h_qp=-J.transpose()*dpref - Jnull.transpose()*M*dqref;
        for(int i=0; i<4; i++){
            lb(i)=max(constr(0,i) - q0(i), -max_speed);
            ub(i)=min(constr(1,i) - q0(i), max_speed);
        }
        //cout << "constr: " << constr << endl;
        //cout << "q0: " << q0.transpose() << endl;
        //cout << "lb: " << lb.transpose() << endl;
        //cout << "ub: " << ub.transpose() << endl;
        // map stuff
        Map<MatrixXd>( &H_qpoases[0], H_qp.cols(), H_qp.rows() ) = H_qp.transpose();
        Map<MatrixXd>( &h_qpoases[0], h_qp.rows(), h_qp.cols() ) = h_qp;
        Map<MatrixXd>( &lb_qpoases[0], lb.rows(), lb.cols() ) = lb;
        Map<MatrixXd>( &ub_qpoases[0], ub.rows(), ub.cols() ) = ub;
        // init qp solver
        if(!is_init[leg]){
            qpOASES::QProblemB tmpSolver(4);
            ikinQpSolver[leg]=tmpSolver;
            qpOASES::Options myOptions;
            myOptions.setToMPC();
            myOptions.printLevel = qpOASES::PL_NONE;
            ikinQpSolver[leg].setOptions(myOptions);
        }
        qpOASES::int_t nWSR = 80;
        ikinQpSolver[leg].init( H_qpoases,h_qpoases,lb_qpoases,ub_qpoases, nWSR, 0 );


        // get solution
        qpOASES::real_t sol_qpoases[4];
        ikinQpSolver[leg].getPrimalSolution(sol_qpoases);
        dq(0)=sol_qpoases[0];
        dq(1)=sol_qpoases[1];
        dq(2)=sol_qpoases[2];
        dq(3)=sol_qpoases[3];

        q0+=dq;

        H_vec=legKinematics(q0, leg);
        H=H_vec[0]*H_vec[1]*H_vec[2]*H_vec[3]*H_vec[4];
        p0=H.block<3, 1>(0, 3);
        dpref=pref-p0;

        if(dpref.norm()<=tol){
            break;
        }
    }
    *err=dpref.norm();

    if(leg==0 && LOG_DATA){
        static ofstream feetRefIkinLogF("feetRefIkinLogF.txt");
        feetRefIkinLogF << pref.transpose() << "\t" << p0.transpose() << endl;
    }
    if(leg==2 && LOG_DATA){
        static ofstream feetRefIkinLogH("feetRefIkinLogH.txt");
        feetRefIkinLogH << pref.transpose() << "\t" << p0.transpose() << endl;
    }
    return q0;
}


Vector3d
Controller :: getCoM()
{

    static Matrix4d mvec=MatrixXd::Identity(4,4);
    static double total_mass;

    CoM<<0,0,0;
    total_mass=0;


    //head
    mvec.block<3,1>(0,3)=masses.block<3,1>(1,0);
    CoM+=masses(0,0)*(HJh_g*mvec).block<3,1>(0,3);
    total_mass+=masses(0,0);
    

    //spine
    for(int i=0; i<6; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,1+i);
        CoM+=masses(0,1+i)*(HJs_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,1+i);
    }

    //FL
    for(int i=0; i<4; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,7+i);
        CoM+=masses(0,7+i)*(HJfl_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,7+i);
    }

    //FR
    for(int i=0; i<4; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,7+i+4);
        CoM+=masses(0,7+i+4)*(HJfr_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,7+i+4);
    }

    //HL
    for(int i=0; i<4; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,7+i+8);
        CoM+=masses(0,7+i+8)*(HJhl_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,7+i+8);
    }

    //HR
    for(int i=0; i<4; i++){
        mvec.block<3,1>(0,3)=masses.block<3,1>(1,7+i+12);
        CoM+=masses(0,7+i+12)*(HJhr_g[i]*mvec).block<3,1>(0,3);
        total_mass+=masses(0,7+i+12);
    }


    CoM/=total_mass;

    return CoM;
}


void
Controller :: getLegJacobians()
{
    legJacob[0]=Jacob(fbck_position.block<4,1>(SPINE_SIZE   ,0), 0);
    legJacob[1]=Jacob(fbck_position.block<4,1>(SPINE_SIZE+4 ,0), 1);
    legJacob[2]=Jacob(fbck_position.block<4,1>(SPINE_SIZE+8 ,0), 2);
    legJacob[3]=Jacob(fbck_position.block<4,1>(SPINE_SIZE+12,0), 3);
}




MatrixXd
Controller :: followCoMReference(MatrixXd comRef)
{
    static Vector2d comReal, comEst, comU;
    static Vector3d comTmp;
    static bool is_init=false;
    static PID PIDx(1,0,0,dt), PIDy;
    MatrixXd feetMod=MatrixXd::Zero(3,4);

    if(is_init==false){
        comU << 0,0;
        PIDy=PIDx;
        is_init=true;
    }

    // get com estimation
    comTmp=getCoM();
    comEst=comTmp.block(0,0,2,1);
    comReal = comEst + comU*0;


    // PID
    comU(0)=PIDx.calc(comRef(0) - comReal(0));
    comU(1)=PIDy.calc(comRef(1) - comReal(1));

    //comU(0)+=(comRef(0) - comReal(0));
    //comU(1)+=(comRef(1) - comReal(1));



    feetMod << -comU(0), -comU(0), -comU(0), -comU(0),
                -comU(1), -comU(1), -comU(1), -comU(1),
                0, 0, 0, 0;
    //cout << feetMod << endl;

    feetMod.block<3,1>(0,0)=Fgird.block<3,3>(0,0).inverse()*feetMod.block<3,1>(0,0);
    feetMod.block<3,1>(0,1)=Fgird.block<3,3>(0,0).inverse()*feetMod.block<3,1>(0,1);
    feetMod.block<3,1>(0,2)=Hgird.block<3,3>(0,0).inverse()*feetMod.block<3,1>(0,2);
    feetMod.block<3,1>(0,3)=Hgird.block<3,3>(0,0).inverse()*feetMod.block<3,1>(0,3);



    return feetMod;
}

















