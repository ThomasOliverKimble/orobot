 #include "controller.hpp"

using namespace std;
using namespace Eigen;



void
Controller :: GetCompass(double data[3])
{	
	compassData[0]=data[0];  
	compassData[1]=data[1];
	compassData[2]=data[2];

}

/* Choose appropriate frequency based on the desired velocity and range of the legs */
void
Controller :: getWalkingFrequency(){
    double front_range=2*ellipse_a(0);
    double hind_range=2*ellipse_a(2);

    freq_walk=walking_forward_velocity/min(front_range, hind_range)*Duty(0);
}

/* Choose appropriate stance starting point to get a maximum possible stance length on the ellipse */
void
Controller :: getSwingStanceEndPoints()
{
    double front_angle=atan2(forVelocity_filtered(1,0), forVelocity_filtered(0,0))*0;
    double hind_angle=atan2(forVelocity_filtered(1,1), forVelocity_filtered(0,1))*0;


    double r_front=ellipse_a(0)*ellipse_b(0)/sqrt(ellipse_a(0)*ellipse_a(0)*sin(front_angle)*sin(front_angle)+
                                                  ellipse_b(0)*ellipse_b(0)*cos(front_angle)*cos(front_angle));
    double r_hind=ellipse_a(2)*ellipse_b(2)/sqrt(ellipse_a(2)*ellipse_a(2)*sin(hind_angle)*sin(hind_angle)+
                                                  ellipse_b(2)*ellipse_b(2)*cos(hind_angle)*cos(hind_angle));

    for(int i=0; i<2; i++){
        stanceStart(0,i)=r_front*cos(front_angle)+midStance(0,i);
        stanceStart(0,i+2)=r_hind*cos(hind_angle)+midStance(0,i+2);
        stanceStart(1,i)=r_front*sin(front_angle)+midStance(1,i);
        stanceStart(1,i+2)=r_hind*sin(hind_angle)+midStance(1,i+2);
        stanceStart(2,i)=midStance(2,i);
        stanceStart(2,i+2)=midStance(2,i+2);
    }
    for(int i=0; i<2; i++){
        stanceEstEnd(0,i)=r_front*cos(front_angle+my_pi)+midStance(0,i);
        stanceEstEnd(0,i+2)=r_hind*cos(hind_angle+my_pi)+midStance(0,i+2);
        stanceEstEnd(1,i)=r_front*sin(front_angle+my_pi)+midStance(1,i);
        stanceEstEnd(1,i+2)=r_hind*sin(hind_angle+my_pi)+midStance(1,i+2);
        stanceEstEnd(2,i)=midStance(2,i);
        stanceEstEnd(2,i+2)=midStance(2,i+2);
    }


}

/* Updates phase of the legs */
void
Controller :: legPhaseDynamics(double dt)
{   
    
    double sumterm;
    double leg_cpg_weight=0.3;
    static MatrixXd legCPGtheta=MatrixXd::Ones(4,2)*0; 
   
    static bool is_init=false;
    if(!is_init){
        legCPGtheta.block<4,1>(0,0)=phShifts.transpose()*2*my_pi;
        legCPGtheta.block<4,1>(0,1)=phShifts.transpose()*2*my_pi; 
        is_init=true;
    }
    
    
    //===================================== LEG CPG ===================================================
	for(int i=0; i<4; i++){
		sumterm=0;
        for(int j=0; j<4; j++){
            sumterm+=leg_cpg_weight*(  (legCPGtheta(j,0) - phShifts(j)*2*my_pi) - (legCPGtheta(i,0) - phShifts(i)*2*my_pi)  )*dt;
        }
        legCPGtheta(i,1)=legCPGtheta(i,0) + 2*my_pi*freq_walk*dt + sumterm;


        legCPGtheta(i,1)=legCPGtheta(i,0) + 2*my_pi*freq_walk*dt;
        legPhase(i)=fmod(legCPGtheta(i,1)/2/my_pi, 1);
        

    }
    legCPGtheta.block<4,1>(0,0)=legCPGtheta.block<4,1>(0,1);    //remember old value
    //============================== DETECT STANCE AND SWING TRANSITIONS =============================
    for(int i=0; i<4; i++){
        if(legPhase(i)<Duty(i/2))
        {
            legs_stance(i)=1;
        }
        else{
            legs_stance(i)=0;
        }

        if(legs_stance_old(i)==1 && legs_stance(i)==0)
            phaseTransitionStanceSwing(i)=1;
        else
            phaseTransitionStanceSwing(i)=0;
        if(legs_stance_old(i)==0 && legs_stance(i)==1)
            phaseTransitionSwingStance(i)=1;
        else
            phaseTransitionSwingStance(i)=0;
    }
    legs_stance_old=legs_stance;
    //================================================================================================

}


/* Returns a point for a corresponding phase on the swing trajectory defined by a besiere spline passing through
initial, middle and final point */
Vector3d 
Controller :: getSwingTrajectory(Vector3d initPoint, Vector3d middlePoint, Vector3d finalPoint, double phase, int leg)
{

    // get baziere parameters (dirty)
    static MatrixXd trAngles(4,4);
    trAngles << trAnglesF0[0], trAnglesF0[1], trAnglesF0[2], trAnglesF0[3], 
                trAnglesF0[0], trAnglesF0[1], trAnglesF0[2], trAnglesF0[3], 
                trAnglesH0[0], trAnglesH0[1], trAnglesH0[2], trAnglesH0[3], 
                trAnglesH0[0], trAnglesH0[1], trAnglesH0[2], trAnglesH0[3];


    static MatrixXd bezierParam(4,2);
    bezierParam  << bezierParamF0[0], bezierParamF0[1],
                    bezierParamF0[0], bezierParamF0[1], 
                    bezierParamH0[0], bezierParamH0[1], 
                    bezierParamH0[0], bezierParamH0[1];


    // initialize beziere variables                             
    static MatrixXd B1(3, 4), B2(3, 4);
    static Vector3d Bcurve(3, 1);
    Transform<double,3,Affine>  tempRot;
    static double bnorm, tB;


    B1.block<3, 1>(0,0)=initPoint;
    B1.block<3, 1>(0,3)=middlePoint;
    bnorm=(B1.block<3, 1>(0,0)-B1.block<3, 1>(0,3)).norm();
 
    tempRot=AngleAxisd(trAngles(leg,2), Vector3d::UnitX())*AngleAxisd(trAngles(leg,0), Vector3d::UnitY());
    B1.block<3, 1>(0,1) = B1.block<3,1>(0,0)+tempRot*Vector3d::UnitZ()*Scaling(bezierParam(leg,1)*bnorm);                  
    B1.block<3, 1>(0,2) << B1(0,3) - bezierParam(leg,0)*(B1(0, 3)-B1(0,0)), 
                            B1(1,3), 
                            B1(2,3);
 


    B2.block<3, 1>(0,0)=middlePoint;
    B2.block<3, 1>(0,3)=finalPoint;
    bnorm=(B2.block<3, 1>(0,0)-B2.block<3, 1>(0,3)).norm();
    tempRot=AngleAxisd(trAngles(leg,3), Vector3d::UnitX())*AngleAxisd(trAngles(leg,1), Vector3d::UnitY());

    B2.block<3, 1>(0,2) = B2.block<3,1>(0,3)+tempRot*Vector3d::UnitZ()*Scaling(bezierParam(leg,1)*bnorm);                        
    B2.block<3, 1>(0,1) << B2(0,0) + bezierParam(leg,0)*(B2(0, 3)-B2(0,0)), 
                            B2(1,0), 
                            B2(2,0);




    // get swing points
    if(phase<0.5){
        tB=phase*2;

        Bcurve= pow(1-tB, 3)*B1.block<3, 1>(0,0) +
                3*pow(1-tB, 2)*tB* B1.block<3, 1>(0,1)  +
                3*pow(tB, 2)*(1-tB)* B1.block<3, 1>(0,2)  +
                pow(tB, 3)*B1.block<3, 1>(0,3); 
    }
    else{
        tB=phase*2-1;

        Bcurve= pow(1-tB, 3)*B2.block<3, 1>(0,0) +
                3*pow(1-tB, 2)*tB* B2.block<3, 1>(0,1)  +
                3*pow(tB, 2)*(1-tB)* B2.block<3, 1>(0,2)  +
                pow(tB, 3)*B2.block<3, 1>(0,3);
    }
    //velFR=(Bcurve - rFR)/dt;
    //rFR=rFR + velFR*dt;

    //legs_stance(1)=0;



    return Bcurve;

} 

/* Takes care of a swing phase of the swinging leg */
bool
Controller :: moveSwingLeg(int leg)
{
    static Vector3d trajPoint;
    static MatrixXd initPoint(3,4), middlePoint(3,4), finalPoint(3,4);
    static bool is_init=false;
    if(!is_init){
        initPoint.block<3,1>(0,0)=(Fgird.inverse()*HJfl_g[4]).block<3,1>(0,3);
        initPoint.block<3,1>(0,1)=(Fgird.inverse()*HJfr_g[4]).block<3,1>(0,3);
        initPoint.block<3,1>(0,2)=(Hgird.inverse()*HJhl_g[4]).block<3,1>(0,3);
        initPoint.block<3,1>(0,3)=(Hgird.inverse()*HJhr_g[4]).block<3,1>(0,3);
        is_init=true;
    }

    if(phaseTransitionStanceSwing(leg)){
            Vector3d tmp=feetReference.block<3,1>(0,leg);
            //if(leg>1){
            //    tmp(0) += -IG-Hgird(0, 3);
            //}
            initPoint.block<3,1>(0,leg)=AngleAxisd(-forTraj(2+3*(leg/2),1)+girdleTraj(2+3*(leg/2),1), Vector3d::UnitZ())*tmp;
            if(leg>1){
                initPoint(0,leg) -= -IG-Hgird(0, 3); 
            }
    }

    middlePoint.block<3,1>(0,leg)=midStance.block<3,1>(0,leg);
    middlePoint(1,leg)+=swing_width(leg);
    middlePoint(2,leg)+=swing_height(leg);

    //finalPoint.block<3,1>(0,leg)=midStance.block<3,1>(0,leg);
    //finalPoint(0,leg)+=ellipse_a;
    finalPoint=stanceStart;

    legs_stance(leg)=0;


    //finalPoint(0)=0.5*Duty(leg/2)/freq_walk*0.2;
    middlePoint(0,leg)=(initPoint(0,leg)+finalPoint(0,leg))/2.;

    //cout  << (HJfl_g[4]).block<3,1>(0,3) << endl << endl;

    
    swingPhase(leg)=(legPhase(leg)-Duty(leg/2))/(1.-Duty(leg/2));
    trajPoint=getSwingTrajectory(initPoint.block<3,1>(0,leg), middlePoint.block<3,1>(0,leg), 
                                        finalPoint.block<3,1>(0,leg), swingPhase(leg), leg);

    if(LOG_DATA){
        static ofstream swingLog("swingLog.txt");
        if(leg==2){
            swingLog << trajPoint.transpose() << endl;
            //cout << "init: " <<initPoint.block<3,1>(0,leg).transpose() << "\t";
            //cout << "middle: " <<middlePoint.block<3,1>(0,leg).transpose() << "\t";
            //cout << "final: " <<stanceStart.block<3,1>(0,leg).transpose() << endl;
        }
    }



    // translate to cancel the shift of hind girdle due to the spine bending 
    if(leg>1){
        trajPoint(0) += -IG-Hgird(0, 3);
        //cout << girdleTraj(5,1) << endl<< endl;
    }

    // rotate to cancel girdle rotation
    feetReference.block<3,1>(0,leg) = AngleAxisd(forTraj(2+3*(leg/2),1)-girdleTraj(2+3*(leg/2),1), Vector3d::UnitZ())*trajPoint;




    if(swingPhase(leg)>=1){
        return true;
    }
    return false;
    
}










