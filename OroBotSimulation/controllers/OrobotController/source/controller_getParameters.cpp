#include "controller.hpp"
#include <iostream>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdlib.h>



using namespace std;
using namespace Eigen;





/* Reads parameters from parameters_ikin.txt file */
bool
Controller :: getParameters()
{
    int i, j;
    stringstream stringstream_file;
    ifstream file_kinematics;
    ifstream file_ikinController;
    ifstream file_animalAnglesWalking;
    ifstream file_animalSpine;
    ifstream file_joystick;
    ifstream file_animalSwingStance;
    ifstream file_masses;
    ifstream file_joysticRec;
    file_masses.open("config/masses.config");
    ifstream file_orobotSpecific;
    file_orobotSpecific.open("config/orobotSpecific.config");
    int numoflines;
    

    file_kinematics.open("config/kinematics.config");
    file_ikinController.open("config/ikinController.config");
    file_joysticRec.open("data/joysticRec.txt");


    file_joystick.open("config/joystick.config");


    

    if(file_kinematics.is_open()) {
        stringstream_file.str(std::string());
        readFileWithLineSkipping(file_kinematics, stringstream_file);

        //spine
        for(i=0; i<SPINE_SIZE; i++){
            stringstream_file>>spine_kin(i);
        }
        //cout<<"spine"<<spine_kin.transpose()<<endl;
        stringstream_file>>girdLoc(0);
        stringstream_file>>girdLoc(1);

        cout << "girdLoc " << girdLoc.transpose() << endl;


        trunk_kin.setZero();
        trunk_kin.block<5,1>(0,0)=spine_kin.block(1,0,5,1);
        //trunk_kin(0)*=0.744329426092252;
        //trunk_kin(4)*=0.325065475273456;
        trunk_kin(0) -= girdLoc(0);
        trunk_kin(4) = girdLoc(1);
        // IG
        IG=trunk_kin.sum();

        cout << "!!!!!!!!!!!!!!!!!!IG!!!!!!!!!!!!!!!!!!!: " << IG << endl;
        // legs
        for(i=0;i<5;i++){
            for(j=0;j<3;j++){
                stringstream_file>>FL_kin(i, j); 
            }
        }    
        cout<<"FL_kin\n"<<FL_kin<<endl;
        for(i=0;i<5;i++){
            for(j=0;j<3;j++){
                stringstream_file>>FR_kin(i, j);
            }
        }   
        cout<<"FR_kin\n"<<FR_kin<<endl;
        for(i=0;i<5;i++){
            for(j=0;j<3;j++){
                stringstream_file>>HL_kin(i, j);
            }
        }   
        cout<<"HL_kin\n"<<HL_kin<<endl;
        for(i=0;i<5;i++){
            for(j=0;j<3;j++){
                stringstream_file>>HR_kin(i, j);
            }
        }       
        cout<<"HR_kin\n"<<HR_kin<<endl;


        for(i=0; i<NUM_MOTORS; stringstream_file>>angSignsCorrIkin2Webots[i], i++);
        for(i=0; i<NUM_MOTORS; stringstream_file>>angShiftsCorrIkin2Webots[i], i++);

        for(i=0; i<NUM_MOTORS; stringstream_file>>angSignsCorrIkin2Robot[i], i++);
        for(i=0; i<NUM_MOTORS; stringstream_file>>angShiftsCorrIkin2Robot[i], i++);

        for(i=0; i<NUM_MOTORS; stringstream_file>>angSignsCorrWebots2Robot[i], i++);
        for(i=0; i<NUM_MOTORS; stringstream_file>>angShiftsCorrWebots2Robot[i], i++);
    }

    if(file_ikinController.is_open()) {
        readFileWithLineSkipping(file_ikinController, stringstream_file);
        //maxSpeed
        stringstream_file>>maxSpeed;

        stringstream_file>>T_trans0;
        T_trans=T_trans0;

        stringstream_file>>Tfilt_angles;

        // trajectory filtering
        stringstream_file>>Tf1;

        //spineCPGscaling
        stringstream_file>>spineCPGscaling(0);
        stringstream_file>>spineCPGscaling(1);
  
         // duty cycles for legs (front and hind)
        for(i=0; i<2; stringstream_file>>Duty(i), i++);      cout<<Duty.transpose()<<endl;

        // phase shift between legs
        for(i=0; i<4; stringstream_file>>phShifts(i), i++);  cout<<phShifts.transpose()<<endl;
            
        // trajectories MID STANCE
        for(i=0; i<3; i++){
            stringstream_file>>midStance(i, 0);
        }
        midStance.block<3,1>(0,1)=midStance.block<3,1>(0,0); midStance(1,1)*=-1;
        for(i=0; i<3; i++){
            stringstream_file>>midStance(i, 2);
        }  
        midStance.block<3,1>(0,3)=midStance.block<3,1>(0,2); midStance(1,3)*=-1;


        // ellipse props
        stringstream_file >> ellipse_a(0); 
        ellipse_a(1)=ellipse_a(0);
        ellipse_a(2)=ellipse_a(0);
        ellipse_a(3)=ellipse_a(0);
        //stringstream_file >> ellipse_a(2); 

        stringstream_file >> ellipse_b(0); 
        ellipse_b(1)=ellipse_b(0);
        ellipse_b(2)=ellipse_b(0);
        ellipse_b(3)=ellipse_b(0);
        //stringstream_file >> ellipse_b(2); ellipse_b(3)=ellipse_b(2);

        //cout << "============================MIDSTANCE ====================================" << endl; 
        //cout << midStance << endl;
        //cout << "============================ELLIPSE ====================================" << endl; 
        //cout << ellipse_a << " b: "<<ellipse_b << endl;
        // swing height FRONT
        stringstream_file >> swing_height(0);
        swing_height(1)=swing_height(0); 
        // swing height HIND
        stringstream_file >> swing_height(2); 
        swing_height(3)=swing_height(2);

        // swing width FRONT
        stringstream_file >> swing_width(0);
        swing_width(1)=swing_width(0); 
        // swing width HIND
        stringstream_file >> swing_width(2); 
        swing_width(3)=swing_width(2);

        // some stupid angles for splines
        for(i=0; i<4; stringstream_file>>trAnglesF0[i], trAnglesF0[i]*=my_pi/180., i++); 
        for(i=0; i<2; stringstream_file>>bezierParamF0[i], i++);
        


        for(i=0; i<4; stringstream_file>>trAnglesH0[i], trAnglesH0[i]*=my_pi/180., i++);   
        for(i=0; i<2; stringstream_file>>bezierParamH0[i], i++);                                                    
        




        


        // constraints
        for(i=0; i<4; stringstream_file>>constrFL(0, i), constrFL(0, i)*=my_pi/180., i++);
        for(i=0; i<4; stringstream_file>>constrFL(1, i), constrFL(1, i)*=my_pi/180., i++);   cout<<constrFL.transpose()<<endl;
        for(i=0; i<4; stringstream_file>>constrFR(0, i), constrFR(0, i)*=my_pi/180., i++);
        for(i=0; i<4; stringstream_file>>constrFR(1, i), constrFR(1, i)*=my_pi/180., i++);   cout<<constrFR.transpose()<<endl;
        for(i=0; i<4; stringstream_file>>constrHL(0, i), constrHL(0, i)*=my_pi/180., i++);
        for(i=0; i<4; stringstream_file>>constrHL(1, i), constrHL(1, i)*=my_pi/180., i++);   cout<<constrHL.transpose()<<endl;
        for(i=0; i<4; stringstream_file>>constrHR(0, i), constrHR(0, i)*=my_pi/180., i++);
        for(i=0; i<4; stringstream_file>>constrHR(1, i), constrHR(1, i)*=my_pi/180., i++);   cout<<constrHR.transpose()<<endl;
        stringstream_file >> constrS;

        /*
        constrF(0, 2)=constrF(0, 2) - my_pi/2;
        constrF(1, 2)=constrF(1, 2) - my_pi/2;
        constrH(0, 3)=constrH(0, 3) - my_pi/2;
        constrH(1, 3)=constrH(1, 3) - my_pi/2;
        */

        // use animal data for front and hind limbs
        stringstream_file>>useAnDF;                          //cout<<useAnDF<<endl;
        stringstream_file>>useAnDH;                          //cout<<useAnDH<<endl;
        stringstream_file>>useAnSP;                          //cout<<useAnSP<<endl;

        

        // fixed initial conditions for iKin
        for(i=0; i<4; stringstream_file>>q0FL(i), q0FL(i)*=my_pi/180., i++);
        for(i=0; i<4; stringstream_file>>q0FR(i), q0FR(i)*=my_pi/180., i++);
        for(i=0; i<4; stringstream_file>>q0HL(i), q0HL(i)*=my_pi/180., i++);
        for(i=0; i<4; stringstream_file>>q0HR(i), q0HR(i)*=my_pi/180., i++);

        cout << "q0FL "<< q0FL.transpose() << endl;
        cout << "q0FR "<< q0FR.transpose() << endl;
        cout << "q0HL "<< q0HL.transpose() << endl;
        cout << "q0HR "<< q0HR.transpose() << endl;
        // lam, M, Cinv and max_dist for iKinDLS
        
        stringstream_file >> lamF(0);
        stringstream_file >> lamF(1);
        stringstream_file >> lamH(0);
        stringstream_file >> lamH(1);

        for(i=0; i<4; stringstream_file>>MF(0, i), i++);
        for(i=0; i<4; stringstream_file>>MF(1, i), i++);
        for(i=0; i<4; stringstream_file>>MF(2, i), i++);
        for(i=0; i<4; stringstream_file>>MF(3, i), i++);   //cout<<MF<<endl;
        for(i=0; i<4; stringstream_file>>MH(0, i), i++);
        for(i=0; i<4; stringstream_file>>MH(1, i), i++);
        for(i=0; i<4; stringstream_file>>MH(2, i), i++);
        for(i=0; i<4; stringstream_file>>MH(3, i), i++);   //cout<<MH<<endl;

        stringstream_file >> max_dist;

        stringstream_file >> ikin_tol; //cout<<ikin_tol<<endl;

        stringstream_file >> ikin_maxIter; //cout<<ikin_maxIter<<endl;

        stringstream_file >> ikin_max_speed; //ikin_max_speed
        ikin_max_speed=ikin_max_speed*dt*my_pi/180.;



        CinvF=lamF(0)/2.*Matrix4d::Identity()+MF.transpose()*MF;
        CinvF=CinvF.inverse().eval();

        CinvH=lamH(0)/2.*Matrix4d::Identity()+MH.transpose()*MH;
        CinvH=CinvH.inverse().eval();
        


    }





    if(file_joystick.is_open()){
        readFileWithLineSkipping(file_joystick, stringstream_file);
        stringstream_file >> stick_r_lim;
        stringstream_file >> joy_walk_max_freq;
        stringstream_file >> joy_walk_min_freq;
        stringstream_file >> joy_walk_speed_change_filt;


        stringstream_file >> disable_crab_walk_lim;
        stringstream_file >> ellipse_small_axis;
        stringstream_file >> posing_joy_y1_rate;
        stringstream_file >> posing_joy_x1_rate;
        stringstream_file >> posing_joy_y2_rate;
        stringstream_file >> posing_head_rate;
        stringstream_file >> posing_head_limit;


    }




    






    if(file_orobotSpecific.is_open()){ cout<<"FILE_OROBOT_SPECIFIC_OPENED"<<endl;
        

        readFileWithLineSkipping(file_orobotSpecific, stringstream_file);
        stringstream_file >> or_walk_freq;
        //FIXED PARAMETERS from TRACKWAYS
        stringstream_file >> or_IGscaling;
        stringstream_file >> or_Ltot;
        

        stringstream_file >> or_Wf;
        stringstream_file >> or_Wh;


        stringstream_file >> or_deltaFH;

        //FREE PARAMETERS (or semifree)
        stringstream_file >> or_spineCPGscaling(0);
        stringstream_file >> or_spineCPGscaling(1);

        stringstream_file >> or_deltaLf;
        stringstream_file >> or_deltaLh;

        stringstream_file >> or_deltaWf;
        stringstream_file >> or_deltaWh;



        stringstream_file >> or_Df;
        stringstream_file >> or_Dh;



        //rewriting leg parameters
        stringstream_file >> or_heightF;
        stringstream_file >> or_heightH;
        stringstream_file >> or_lift_height_F;
        stringstream_file >> or_lift_height_H;

        stringstream_file >> or_stance_end_lift;
        stringstream_file >> or_stance_end_lift_exp_const;


        stringstream_file >> or_phase_FL;
        stringstream_file >> or_deltaphi;

        stringstream_file >> legs_offset;
        
        stringstream_file >> or_ROLLvsYAW;

        for(i=0; i<4; stringstream_file>>or_MF(0, i), i++);
        for(i=0; i<4; stringstream_file>>or_MF(1, i), i++);
        for(i=0; i<4; stringstream_file>>or_MF(2, i), i++);
        for(i=0; i<4; stringstream_file>>or_MF(3, i), i++);   cout<<"or_MF: \n"<<or_MF<<endl<<endl;
        for(i=0; i<4; stringstream_file>>or_MH(0, i), i++);
        for(i=0; i<4; stringstream_file>>or_MH(1, i), i++);
        for(i=0; i<4; stringstream_file>>or_MH(2, i), i++);
        for(i=0; i<4; stringstream_file>>or_MH(3, i), i++);   cout<<"or_MH: \n"<<or_MH<<endl<<endl;


    }

    if(file_masses.is_open()){
        readFileWithLineSkipping(file_masses, stringstream_file);
        for(i=0; i<23; i++){
            stringstream_file >> masses(0, i);  // mass
            stringstream_file >> masses(1, i);  // coordinates
            stringstream_file >> masses(2, i);
            stringstream_file >> masses(3, i);
            //cout<<i<<endl;
        }
        //cout<<masses.transpose()<<endl;
    }

    return true;
    
}
