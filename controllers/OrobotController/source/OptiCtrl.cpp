#include "OptiCtrl.hpp"


#include <sstream>

#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <time.h>       /* time */
#define nParameters 3
#define nSettings 3
#define nFitness 1

static stringstream filename;



extern int USE_FEET, RECORD_VIDEOS;


OptiCtrl :: OptiCtrl()
{
    cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!nParameters!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << nParameters << endl;
	t_total=0;


	// init fitness
	fitfun[0]=0;

};

/* Initializes optimization */
void
OptiCtrl :: optimizationInit(Controller *controller, RobotSim *orobot)
{
    // get pointer to the webots opti stuff
    
    if(opti){
        // get names of parameters and settings
        for(int i=0; i<nParameters; i++)
        {
        string str="P";
            stringstream s;
                    s << "P";
                    s << i; cout<<s.str()<<endl;
            params_names.push_back(string(s.str().c_str()));
        }
        for(int i=0; i<nSettings; i++)
        {
            stringstream s("");
                    s << "S";
                    s << i;
            settings_names.push_back(s.str());
        }

        // get parameters
        Task::Parameter p;
        for(unsigned int i=0; i<params_names.size(); i++)
        {
            if (opti.Parameter(params_names[i],p))
                {
                params.push_back(p.value());
            }
            else
            {
                cerr << "optimization parameter ' " << params_names[i] << " ' not found! " << endl;
            }
        }

        // get settings
        string param;
        double val;

        for(unsigned int i=0; i<settings_names.size(); i++)
        {
            if (opti.Setting(settings_names[i], param))
            {
                stringstream s(param);
                s >> val;
                settings.push_back(val); cout<<settings[i]<<endl;
            }
            else
            {
                cerr << "optimization setting ' " << settings_names[i] << " ' not found! " << endl;
            }
        }

        /////////////////////////////////// modify stuff ////////////////////////////////////////////
        
        // ======================== MAIN SEARCH PARAMETERS ============================================
        // SPINE osc
        controller->or_spineCPGscaling(0)=params[0];
        controller->or_spineCPGscaling(1)=params[0];

        // gait height
        controller->or_heightF=params[1];
        controller->or_heightH=params[1];

        // ROLL vs YAW
        controller->or_ROLLvsYAW=params[2];
        



        // ======================== FOOT STIFFNESS INVESTIGATION ============================================

        // stiffness and offset
        /* controller->legs_offset = params[3];
        orobot->set2segFeetParam(params[4], 0.25*params[4]); */

        ///////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////// modify stuff with settings ////////////////////////////////////////////

        // GAIT PARAMETERS
        

        controller->or_walk_freq=settings[1];

        controller->or_Df=settings[2];
        controller->or_Dh=settings[2];


        t_total=1/controller->or_walk_freq*settings[0];






        ///////////////////////////////////////////////////////////////////////////////////////////





    }
}


/* Ends optimization */
void
OptiCtrl :: optimizationEnd(RobotSim *orobot)
{
    vector<double> fit;
    vector<string> fitness_names;
    map<string, double> fitness;

    // fitness variables
    for(int i=0; i<nFitness; i++){
    	fit.push_back(fitfun[i]);
    }
    for(int i=0; i<nFitness; i++)
    {
        stringstream s("");
                s << "F";
                s << i;
        fitness_names.push_back(s.str());
    }


    // pack them
    for(unsigned int i=0; i<fit.size(); i++)
	{
		fitness[fitness_names[i]] = fit[i];
	}

    // Send the actual response
	if (opti.Setting("optiextractor"))
	{
		cout << "Not sending response, because in optiextractor mode! Enjoy the show!" << endl;
	}
	else
	{
		opti.Respond(fitness);
	}




    // cout if not in optimization
	if(opti.Setting("optiextractor") || !opti)
	{
		cout << "Fitness: " << fitfun[0] <<"\t"<< fitfun[1] <<"\t"<< fitfun[2] <<"\t"<< fitfun[0]*fitfun[1]*fitfun[2]<< endl;
	}



	// quit simulation
	if (opti && !opti.Setting("optiextractor"))
		orobot->killSimulation();
}

/* Checks stopping criterion */
int
OptiCtrl :: optimizationShouldWeStopIt(double timestep)
{
    static double t_stop=0;
    t_stop+=timestep;
    //optimization::Webots &opti = optimization::Webots::Instance();
    int crit=0;
    if(opti){ 
        if(t_stop>t_total){
            crit=1;
        }

    }

    return crit;
}

/* Updates internal time and runs state machine */
int
OptiCtrl :: OptiStep(double dt, Controller *controller, RobotSim *orobot)
{
    static bool startedRecording=false, stoppedRecording=false;
    if(opti){
    	static double t_run=0, t_timeout=2;
    	t_run+=dt;
    	

    	// END SIMULATION & OPTIMIZATION
    	if(t_run>=t_total){

    		optimizationEnd(orobot);
    		return -1;
    	}
        OptiLog(controller, orobot);


        string filename_video, ext(".mp4");
        //filename_video << filename.rdbuf();
        filename_video = filename.str();
        filename_video.erase(filename_video.end()-8,filename_video.end());
        filename_video+=ext;

        if(RECORD_VIDEOS){
            if(t_run>(2*1/controller->or_walk_freq)){
                if(startedRecording==false){
                    orobot->startVideoRecording(filename_video, 640, 360);
                    startedRecording=true;
                }
            }

            if(t_run>(4*1/controller->or_walk_freq)){
                if(stoppedRecording==false){
                    orobot->stopVideoRecording();
                    stoppedRecording=true;
                }
            }


        }
        




    }

	return 1;
}


/* Logs data into txt files */
void
OptiCtrl :: OptiLog(Controller *controller, RobotSim *orobot)
{
    
    static bool is_init=false;
    
    static unsigned long int counter=0;
    if(!is_init){
        filename << "/data/thorvat/orobot/";
        filename << "set_";
        for(int i=0; i<nSettings; i++){
            filename << settings[i];
            filename << "_";
        }

        filename << "par_";
        for(int i=0; i<nParameters; i++){
            filename << params[i];
            filename << "_";
        }
        srand (time(NULL));
        filename << rand();
        filename << ".optilog";

        is_init=true;
    }   
    
    static ofstream optilog(filename.str());

    static MatrixXd fbck_position_old = controller->fbck_position.transpose();
    static MatrixXd fbck_torques_old = controller->fbck_torques.transpose();
    static MatrixXd forRPY_old = controller->forRPY.transpose();
    static MatrixXd feetGPS_old = controller->feetGPS;
    static MatrixXd gpsPos_old = controller->gpsPos.transpose();
    static MatrixXd legTrackingError_old = controller->legTrackingError.transpose();
    static double fgird_orient = controller->girdleTraj(2,1);
    static double hgird_orient = controller->girdleTraj(5,1);



    // ====================== LOG EVERY SECOND SAMPLE =============================================
    /*if((counter%2)==0)
    {
        // LOG TIME
        optilog << controller->t <<"\t";

        // LOG JOINT ANGLES
        optilog << (controller->fbck_position.transpose() + fbck_position_old)/2. <<"\t";
        
        // LOG JOINT TORQUES
        optilog << (controller->fbck_torques.transpose() + fbck_torques_old)/2. <<"\t";
        
        // LOG IMU
        optilog << (controller->forRPY.transpose() + forRPY_old)/2. <<"\t";
        
        // LOG GPS of FRONT GIRDLE and FEET
        optilog << (controller->gpsPos.transpose() + gpsPos_old)/2. <<"\t";
        optilog << (controller->feetGPS.block<3,1>(0,0).transpose() + feetGPS_old.block<3,1>(0,0).transpose())/2. <<"\t";
        optilog << (controller->feetGPS.block<3,1>(0,1).transpose() + feetGPS_old.block<3,1>(0,1).transpose())/2. <<"\t";
        optilog << (controller->feetGPS.block<3,1>(0,2).transpose() + feetGPS_old.block<3,1>(0,2).transpose())/2. <<"\t";
        optilog << (controller->feetGPS.block<3,1>(0,3).transpose() + feetGPS_old.block<3,1>(0,3).transpose())/2. <<"\t";

        // LOG LEG PHASES
        optilog << controller->legPhase.transpose() << "\t";

        // LOG LEG TRACKING ERROR
        optilog << (controller->legTrackingError.transpose() + legTrackingError_old)/2. << "\t"; 

        // GIRDLE ORIENTATIONS
        optilog << (controller->girdleTraj(2,1) + fgird_orient)/2.<< "\t" << (controller->girdleTraj(5,1) + hgird_orient)/2. ;

        optilog << endl;
    }
    


    counter++;
    fbck_position_old=controller->fbck_position.transpose();
    fbck_torques_old=controller->fbck_torques.transpose();
    forRPY_old=controller->forRPY.transpose();
    gpsPos_old=controller->gpsPos.transpose();
    feetGPS_old=controller->feetGPS;
    legTrackingError_old = controller->legTrackingError.transpose();
    fgird_orient = controller->girdleTraj(2,1);
    hgird_orient = controller->girdleTraj(5,1);*/

    // ====================== LOG EVERY SAMPLE =============================================

    // LOG TIME
    optilog << controller->t <<"\t";

    // LOG JOINT ANGLES
    optilog << controller->fbck_position.transpose() <<"\t";
    
    // LOG JOINT TORQUES
    optilog << controller->fbck_torques.transpose() <<"\t";
    
    // LOG IMU
    optilog << controller->forRPY.transpose() <<"\t";
    
    // LOG GPS of FRONT GIRDLE and FEET
    optilog << controller->gpsPos.transpose() <<"\t";
    optilog << controller->feetGPS.block<3,1>(0,0).transpose() <<"\t";
    optilog << controller->feetGPS.block<3,1>(0,1).transpose() <<"\t";
    optilog << controller->feetGPS.block<3,1>(0,2).transpose() <<"\t";
    optilog << controller->feetGPS.block<3,1>(0,3).transpose() <<"\t";

    // LOG LEG PHASES
    optilog << controller->legPhase.transpose() << "\t";

    // LOG LEG TRACKING ERROR
    optilog << controller->legTrackingError.transpose() << "\t"; 

    // GIRDLE ORIENTATIONS
    optilog << controller->girdleTraj(2,1) << "\t" << controller->girdleTraj(5,1)  << "\t";

    // GRF
    optilog << controller->GRF.block<3,1>(0,0).transpose() << "\t";
    optilog << controller->GRF.block<3,1>(0,1).transpose() << "\t";
    optilog << controller->GRF.block<3,1>(0,2).transpose() << "\t";
    optilog << controller->GRF.block<3,1>(0,3).transpose() << "\t";


    optilog << endl;




}






