#include "controller.hpp"

using namespace std;
using namespace Eigen;
using namespace dlib;
extern int IS_SIMULATION, IS_PLEUROBOT, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, SPINE_COMPENSATION_WITH_FEEDBACK, USE_REFLEXES, LOG_DATA;

// ---------------------------------------------- SETUP OPTIMIZATION ----------------------------------------------------------
typedef matrix<double,0,1> column_vector;


class optimizer_spineOscillations
{
	public:
	    optimizer_spineOscillations (
	        Controller *ctrl_input, Vector3d old_solution_input
	    )
	    {
	        ctrl=ctrl_input;
	        old_solution=old_solution_input;
	    }

	    double operator() ( const column_vector& x) const
	    //double operator() ( const double& x) const
	    {
	        
	    	MatrixXd q_trunk(4,1);
	    	//q_trunk=ctrl->q0_trunk_from_spline*x(0) + MatrixXd::Constant(4,1,x(1)); 
	    	q_trunk=ctrl->q0_trunk_from_spline*x(0) + MatrixXd::Constant(4,1,x(1))+ VectorXd::LinSpaced(4,-1,1)*x(2);

	    	// rotate front girdle fore x(1)
	    	Vector3d fgird_pos; 
	    	fgird_pos(0)=ctrl->girdleTraj(0,1);
	    	fgird_pos(1)=ctrl->girdleTraj(1,1);
	    	fgird_pos(2)=ctrl->girdleTraj(2,1);


	    	Vector3d hgird_pos=ctrl->trunkForwardKinematics(fgird_pos, q_trunk);
	    	
	    	// find minimal distance of the hind girdle form the stored trajectory
	    	double mindist[2]={9999,9999}, dist; 
	    	int indx[2]={0,0};
	    	for(int i=30; i<200; i++){
	    		dist=sqrt((ctrl->forTrajHist(0,i)-hgird_pos(0))*(ctrl->forTrajHist(0,i)-hgird_pos(0)) + 
	    				  (ctrl->forTrajHist(1,i)-hgird_pos(1))*(ctrl->forTrajHist(1,i)-hgird_pos(1)));
	    		if(dist<mindist[0]){
	    			mindist[1]=mindist[0];
	    			mindist[0]=dist;
	    			indx[1]=indx[0];
	    			indx[0]=i;
	    		}
	    	}
	    	// calculate actual distance from the curve
	    	double a=mindist[0], b=mindist[1], c,c1,c2,v;
	    	c=sqrt(  pow(ctrl->forTrajHist(0,indx[0])-ctrl->forTrajHist(0,indx[1]), 2)  + 
	    			 pow(ctrl->forTrajHist(1,indx[0])-ctrl->forTrajHist(1,indx[1]), 2) );
	    	c1=(a*a-b*b+c*c)/(2*c);
	    	c2=c-c1;
	    	v=sqrt(a*a-c1*c1);



	    	// cost function
	    	double f1, f2, f3, f4;
    		
    		// distance from the curve
	    	f1 = 1000*v;

	    	// intrusivness of the solution -> scaling close to 1, offset close to 0
	    	f2 = 1000*((x(0)-1)*(x(0)-1) + x(1)*x(1)+x(2)*x(2)); 

	    	// hind girdle angle
	    	//f3 = 0*(hgird_pos(2) - ctrl->girdleCpgOutput(1))*(hgird_pos(2) - ctrl->girdleCpgOutput(1));
	    	double hgirdAngRef;
	    	hgirdAngRef=ctrl->forTraj(5,1) + ctrl->girdleCpgOutput(1);
    		f3 = 1000*(hgird_pos(2) - hgirdAngRef)*(hgird_pos(2) - hgirdAngRef);
    		
    		// distance from the old solution
	    	f4= 10000*((x(0)-old_solution(0))*(x(0)-old_solution(0)) + 
    				(x(1)-old_solution(1))*(x(1)-old_solution(1)) +
    				(x(2)-old_solution(2))*(x(2)-old_solution(2))   );
	    	   	

	        return f1 + f2 + f3 + f4;
	    }

	private:
	    Controller *ctrl;
	    Vector3d old_solution;
};

// ---------------------------------------------- FUNCTIONS ----------------------------------------------------------

/* handles walking and all subcontrollers */
void 
Controller :: walkingStateMachine()
{



	// get leg phases (accounts for duty cycles and leg phase offsets)
	legPhaseDynamics(dt);

	// move swing leg
	for(int i=0; i<4; i++){
		if(legs_stance(i)==0){
			moveSwingLeg(i);
		}
	}


	// get girdle trajectories  
	girdleTrajectories(walking_forward_velocity, walking_angular_velocity);


	// frame of reference velocities
	forVelocity(0,0)=(forTraj(0,1)-forTraj(0,0))/dt;
    forVelocity(1,0)=(forTraj(1,1)-forTraj(1,0))/dt;
    forVelocity(2,0)=0;
    forVelocity(0,1)=(forTraj(3,1)-forTraj(3,0))/dt;
    forVelocity(1,1)=(forTraj(4,1)-forTraj(4,0))/dt;
    forVelocity(2,1)=0;
    forAngularVelocity(0)=(forTraj(2,1)-forTraj(2,0))/dt;
    forAngularVelocity(1)=(forTraj(5,1)-forTraj(5,0))/dt;

    forVelocity.block<3,1>(0,0)=AngleAxisd(walkingDirection, Vector3d::UnitZ())*
    							AngleAxisd(-forTraj(2), Vector3d::UnitZ())*forVelocity.block<3,1>(0,0);
	forVelocity.block<3,1>(0,1)=AngleAxisd(walkingDirection, Vector3d::UnitZ())*
    							AngleAxisd(-forTraj(5), Vector3d::UnitZ())*forVelocity.block<3,1>(0,1);

	// get front girdle velocities in the body (girdle) frame
    girdleVelocity(0,0)=(girdleTraj(0,1)-girdleTraj(0,0))/dt;
    girdleVelocity(1,0)=(girdleTraj(1,1)-girdleTraj(1,0))/dt;
    girdleVelocity(2,0)=0;
    girdleVelocity.block<3,1>(0,0)=AngleAxisd(walkingDirection, Vector3d::UnitZ())*AngleAxisd(-girdleTraj(2,1), Vector3d::UnitZ())*girdleVelocity.block<3,1>(0,0);
    girdleAngularVelocity(0)=(girdleTraj(2,1)-girdleTraj(2,0))/dt;

    // move front girdle
	moveGirdle(girdleVelocity.block<3,1>(0,0), girdleAngularVelocity(0), 0);
	//moveGirdle(forVelocity.block<3,1>(0,0), forAngularVelocity(0), 0);

	// get hind girdle velocities in the body (girdle) frame
	girdleVelocity(0,1)=(girdleTraj(3,1)-girdleTraj(3,0))/dt;
    girdleVelocity(1,1)=(girdleTraj(4,1)-girdleTraj(4,0))/dt;
    girdleVelocity(2,1)=0;
    girdleVelocity.block<3,1>(0,1)=AngleAxisd(walkingDirection, Vector3d::UnitZ())*AngleAxisd(-girdleTraj(5,1), Vector3d::UnitZ())*girdleVelocity.block<3,1>(0,1);
    girdleAngularVelocity(1)=(girdleTraj(5,1)-girdleTraj(5,0))/dt;

    // move hind girdle
	moveGirdle(girdleVelocity.block<3,1>(0,1),girdleAngularVelocity(1), 1);
	//moveGirdle(forVelocity.block<3,1>(0,1),forAngularVelocity(1), 1);

	girdleVelocity_filtered=pt1_vec(girdleVelocity, girdleVelocity_filtered, Tf1, dt);
	girdleAngularVelocity_filtered=pt1_vec(girdleAngularVelocity, girdleAngularVelocity_filtered, Tf1, dt);
	forVelocity_filtered=pt1_vec(forVelocity, forVelocity_filtered, Tf1, dt);
	forAngularVelocity_filtered=pt1_vec(forAngularVelocity, forAngularVelocity_filtered, Tf1, dt);
}

/* Translates and rotates a girdle by using a correspoinging leg pair while in stance */
void 
Controller :: moveGirdle(Vector3d girdleVelocity, double girdleAngularVelocity, int girdleNum)
{
	static Transform<double,3,Affine> Girdle_transformation;
	//Girdle_transformation = AngleAxisd(-girdleAngularVelocity*dt, Vector3d::UnitZ()) * Translation3d(-girdleVelocity(0)*dt, -girdleVelocity(1)*dt, -girdleVelocity(2)*dt);
	Girdle_transformation = AngleAxisd(-girdleAngularVelocity*dt, Vector3d::UnitZ()) * Translation3d(-girdleVelocity*dt);

	for(int i=girdleNum*2; i<2*(1+girdleNum); i++){
		if(legs_stance(i)){
			feetReference.block<3,1>(0,i) = Girdle_transformation*feetReference.block<3,1>(0,i);

			// shift up towards the end of stance

			feetReference(2,i)=stanceStart(2,i)+or_stance_end_lift*swing_height(i) * exp(or_stance_end_lift_exp_const * legPhase(i)/Duty(i/2)) /
													exp(or_stance_end_lift_exp_const);


		}
	}
	//cout << feetReference << endl;
}


/* calculates girdle trajctories / velocities for given input commands - linear and angular velocity of the front girdle */
void
Controller :: girdleTrajectories(double v, double w)
{	

	// INITIALIZE
	MatrixXd tmp_traj(3,100);
	static bool is_init=false;
	static double mindisp=0.004;
	static MatrixXd q_trunk(SPINE_SIZE,1);
	static Vector3d old_solution;
	if(!is_init){
		// initialize trajectory container
		for(int i=0; i<200; i++){
			forTrajHist(0,i)=-i/75.*IG;
			forTrajHist(1,i)=0;
			forTrajHist(2,i)=0;
		}
		old_solution << 1,0,0;
	}

	// ------------------------------------------ REMEMBERING OLD STATES AND FGIRD TRAJECTORY -------------------------------
	// remember old state

	forTraj.block<6,1>(0,0)=forTraj.block<6,1>(0,1);
    girdleTraj.block<6,1>(0,0)=girdleTraj.block<6,1>(0,1);

    // move frame of reference
    forTraj(0,1)=forTraj(0,0)+v*cos(forTraj(2,0))*dt;
    forTraj(1,1)=forTraj(1,0)+v*sin(forTraj(2,0))*dt;
    forTraj(2,1)=forTraj(2,0)+w*dt;

    // remember FOR trajectory - update if displacement is greater than mindelta
    static double disp;
    disp =sqrt((forTraj(0,1)-forTrajHist(0,0))*(forTraj(0,1)-forTrajHist(0,0))+(forTraj(1,1)-forTrajHist(1,0))*(forTraj(1,1)-forTrajHist(1,0)));
    if(disp>mindisp){
    	tmp_traj=forTrajHist.block<3,199>(0,0);
        forTrajHist.block<3,199>(0,1)=tmp_traj;
    	forTrajHist.block<3,1>(0,0)=forTraj.block<3,1>(0,0);
    }
 	

    // ------------------------------------------ RUNNING GIRDLE CPG AND SPLINING STUFF -------------------------------
 	// update girdle oscillations
 	//girdleOscillations();
 	FixedGirdleOscillations();
 	// spline parameter
 	static MatrixXd t_spline(1, 6);
 	static MatrixXd p_spline(2, t_spline.size());
 	if(!is_init)
 	{
		// spline parameter
 		double spinecumsum=trunk_kin(0);
		t_spline(0)=0;
		for(int i=0; i<4; i++)
		{
			t_spline(i+1)=spinecumsum/IG;
			spinecumsum+=trunk_kin(i+1);
		}
		t_spline(5)=spinecumsum/IG;
	}
	// spline points and vectors
	static Vector2d p0_spline, p1_spline, m0_spline, m1_spline;
	//p0_spline=girdleTraj.block(0,0,2,1)*0; 
	//p1_spline << -IG, 0;
	p0_spline=forTraj.block(0,0,2,1);
	p1_spline=forTraj.block(3,0,2,1); 

	//girdleCpgOutput*=(1-abs(gamma));
	m0_spline(0)=-cos(forTraj(2,0) + girdleCpgOutput(0));
	m0_spline(1)=-sin(forTraj(2,0) + girdleCpgOutput(0));
	m1_spline(0)=-cos(forTraj(5,0) + girdleCpgOutput(1));
	m1_spline(1)=-sin(forTraj(5,0) + girdleCpgOutput(1));

 	// get basic angles from spline points
 	static double const scl1=0.1, scl2=0.1;
	p_spline=hermiteSpline(p0_spline, p1_spline, scl1*m0_spline, scl2*m1_spline, t_spline);
	for(int i=0; i<4; i++){
		p0_spline=p_spline.block(0, i+1, 2, 1) - p_spline.block(0, i  , 2, 1);
		p1_spline=p_spline.block(0, i+2, 2, 1) - p_spline.block(0, i+1, 2, 1);
		q0_trunk_from_spline(i)=copysign(			
					SafeAcos(p0_spline.dot(p1_spline)/p0_spline.norm()/p1_spline.norm()),
					p0_spline(0)*p1_spline(1) - p1_spline(0)*p0_spline(1)
					);
	}


    girdleTraj(0,1)=forTraj(0,1);
    girdleTraj(1,1)=forTraj(1,1);
    girdleTraj(2,1)=forTraj(2,1) + girdleCpgOutput(0);




    // ----------------------------------------------------- OPTIMIZATION ---------------------------------------------
   	// --------------- find_optimal_parameters --------------------     
   	static column_vector starting_point(3), x_lower(3), x_upper(3);
   	static bool is_init_opt=false;

	if(!is_init_opt){
		is_init_opt=true;
		starting_point=1,0, 0;
		x_lower=0.5, -1, -1;
		x_upper=1.5, 1, 1;
	}
	
    find_optimal_parameters(
		0.025,														//double initial_search_radius,
		0.0000001,														//double eps,
		800,														//const unsigned int max_f_evals,
		starting_point,												//matrix<double,0,1>& x,
		x_lower,												//const matrix<double,0,1>& x_lower,
		x_upper,												//const matrix<double,0,1>& x_upper,
		optimizer_spineOscillations(this, old_solution)		            	//const funct& f
    );

    old_solution(0)=starting_point(0);
    old_solution(1)=starting_point(1);
    old_solution(2)=starting_point(2);
    //q_trunk.block(2,0,4,1)=q0_trunk_from_spline*starting_point(0) + MatrixXd::Constant(4,1,starting_point(1));  
    q_trunk.block(2,0,4,1)=q0_trunk_from_spline*starting_point(0) + 
    						MatrixXd::Constant(4,1,starting_point(1)) + 
							VectorXd::LinSpaced(4,-1,1)*starting_point(2); 
    

    // --------------- update kinematics -------------------- 

    girdleTraj.block<3,1>(3,1)=trunkForwardKinematics(girdleTraj.block<3,1>(0,1), q_trunk.block(2,0,4,1));
    // find minimal distance of the hind girdle form the stored trajectory
	double mindist[2]={9999,9999}, dist; 
	static int indx[2]={0,0};
	for(int i=50; i<200; i++){
		dist=sqrt((forTrajHist(0,i)-girdleTraj(3))*(forTrajHist(0,i)-girdleTraj(3)) + 
				  (forTrajHist(1,i)-girdleTraj(4))*(forTrajHist(1,i)-girdleTraj(4)));
		if(dist<mindist[0]){
			mindist[1]=mindist[0];
			mindist[0]=dist;
			indx[1]=indx[0];
			indx[0]=i;
		}
	}

	// calculate actual distance from the curve
	double a=mindist[0], b=mindist[1], c, c1, c2;
	c=sqrt(  pow(forTrajHist(0,indx[0])-forTrajHist(0,indx[1]), 2)  + 
			 pow(forTrajHist(1,indx[0])-forTrajHist(1,indx[1]), 2) );
	c1=(a*a-b*b+c*c)/(2*c);
	c2=c-c1;



    // get a new hgird position and orientation
    forTraj(3,1)=girdleTraj(3,1);
    forTraj(4,1)=girdleTraj(4,1);
    forTraj(5,1)=(c-c1)/c*forTrajHist(2,indx[0])  +  (c-c2)/c*forTrajHist(2,indx[1]);


    for(int i=0; i<6; i++){
    	qs(i)=q_trunk(i);
    }
    qs(0)=-(forTraj(2,1)-girdleTraj(2,1))/2;
    qs(1)=-(forTraj(2,1)-girdleTraj(2,1))/2;

    qs(6)=(forTraj(5,1)-girdleTraj(5,1))/2;
    qs(7)=(forTraj(5,1)-girdleTraj(5,1))/2;

    //cout << girdleTraj.block<6,1>(0,1).transpose() << "\t est:" << qs(0)*2 << endl;
    if(!is_init){
		is_init=true;
	}

	if(LOG_DATA){
		static ofstream girdleTrajLog("./data/girdleTrajLog.txt");
	    girdleTrajLog << girdleTraj.block<6,1>(0,0).transpose()<<"\t"<<girdleTraj.block<6,1>(0,1).transpose()<<endl;
		static ofstream spine_angles("./data/spine_angles.txt");
		spine_angles << qs.transpose() << endl;
	}

}


/* predicts future trajectories over prediction horizon for MPC or anything else*/
std::vector<Matrix<double,3,4>> 
Controller :: predictTrajectories(int N, double time_step, MatrixXd *predLegPhase_in)
{

	std::vector<Matrix<double,3,4>> predictedFootsteps;
	predictedFootsteps.resize(N);

	double omega=2*my_pi*freq_walk;
	//predLegPhase.resize(N,4);
	MatrixXd predLegPhase(N,4);
	predLegPhase.block(0,0,1,4)=legPhase.transpose();

	// initial (current) points 
	predictedFootsteps[0].block<3,1>(0,0)=AngleAxisd(-forTraj(2,1)+girdleTraj(2,1), Vector3d::UnitZ())*feetReference.block<3,1>(0,0);
	predictedFootsteps[0].block<3,1>(0,1)=AngleAxisd(-forTraj(2,1)+girdleTraj(2,1), Vector3d::UnitZ())*feetReference.block<3,1>(0,1);
	predictedFootsteps[0].block<3,1>(0,2)=AngleAxisd(-forTraj(5,1)+girdleTraj(5,1), Vector3d::UnitZ())*feetReference.block<3,1>(0,2);
	predictedFootsteps[0].block<3,1>(0,3)=AngleAxisd(-forTraj(5,1)+girdleTraj(5,1), Vector3d::UnitZ())*feetReference.block<3,1>(0,3);

	// starting point for stance phase
	MatrixXd stanceStartingPoint(3,4);

	//stanceStartingPoint.block<3,1>(0,0)=AngleAxisd(forTraj(2,1)-girdleTraj(2,1), Vector3d::UnitZ())*trPointsFL.block<3,1>(0,0);
	//stanceStartingPoint.block<3,1>(0,1)=AngleAxisd(forTraj(2,1)-girdleTraj(2,1), Vector3d::UnitZ())*trPointsFR.block<3,1>(0,0);
	//stanceStartingPoint.block<3,1>(0,2)=AngleAxisd(forTraj(5,1)-girdleTraj(5,1), Vector3d::UnitZ())*trPointsHL.block<3,1>(0,0);
	//stanceStartingPoint.block<3,1>(0,3)=AngleAxisd(forTraj(5,1)-girdleTraj(5,1), Vector3d::UnitZ())*trPointsHR.block<3,1>(0,0);
	
	//stanceStartingPoint.block<3,1>(0,0)=trPointsFL.block<3,1>(0,0);
	//stanceStartingPoint.block<3,1>(0,1)=trPointsFR.block<3,1>(0,0);
	//stanceStartingPoint.block<3,1>(0,2)=trPointsHL.block<3,1>(0,0);
	//stanceStartingPoint.block<3,1>(0,3)=trPointsHR.block<3,1>(0,0);

	//stanceStartingPoint.block<3,1>(0,0)=stanceStart.block<3,1>(0,0);
	//stanceStartingPoint.block<3,1>(0,1)=stanceStart.block<3,1>(0,0);
	//stanceStartingPoint.block<3,1>(0,2)=stanceStart.block<3,1>(0,0);
	//stanceStartingPoint.block<3,1>(0,3)=stanceStart.block<3,1>(0,0);
	stanceStartingPoint=stanceStart;
	Transform<double,3,Affine> Girdle_transformation;


	for(int j=0; j<4; j++){
		if(predLegPhase(0,j)>Duty(j/2)){
			predictedFootsteps[0].block<3,1>(0,j) << -9999,
							                   -9999,
			   				                   -9999;
		}
	}


	
	// LOOK INTO THE FUTURE
	

	for(int i=1; i<N; i++){

		// run legPhaseDynamics and determine if swing or stance
		predLegPhase.block(i,0,1,4)=predLegPhase.block(i-1,0,1,4)+freq_walk*time_step*MatrixXd::Ones(1,4);
		for(int j=0; j<4; j++){
			predLegPhase(i,j)=predLegPhase(i,j)>1 ? (0+(predLegPhase(i,j)-1)) : predLegPhase(i,j);
		}

		// when in stance move the leg with current speed profiles
		for(int j=0; j<4; j++){
			
			// ignore if in swing phase										
			if(predLegPhase(i,j)>Duty(j/2)){
				predictedFootsteps[i].block<3,1>(0,j) << -9999,
								                   -9999,
				   				                   -9999;
               	continue;
			}
			// if comming back from the swing phase (landing)
			if(predictedFootsteps[i-1](2,j)==-9999){
				predictedFootsteps[i].block<3,1>(0,j)=stanceStartingPoint.block<3,1>(0,j);
				continue;
			}

			// otherwise, move leg with a current speed
			//Girdle_transformation = AngleAxisd(-girdleAngularVelocity_filtered(j/2)*time_step, Vector3d::UnitZ()) * 
			//									Translation3d((-girdleVelocity_filtered.block<3,1>(0,j/2))*time_step);
			Girdle_transformation = AngleAxisd(-forAngularVelocity_filtered(j/2)*time_step, Vector3d::UnitZ()) * 
												Translation3d((-forVelocity_filtered.block<3,1>(0,j/2))*time_step);

			predictedFootsteps[i].block<3,1>(0,j)=Girdle_transformation*predictedFootsteps[i-1].block<3,1>(0,j);
			//predictedFootsteps[i-1].block<3,1>(0,j)=AngleAxisd(forTraj(2,1)-girdleTraj(2,1), Vector3d::UnitZ())*predictedFootsteps[i-1].block<3,1>(0,j);
		}
		
	}




	*predLegPhase_in=predLegPhase;
	return predictedFootsteps;
}




/* Calculate girdle oscillations to follow leg movements */
void
Controller :: girdleOscillations()
{


	int N=60;
	double pred_dt=0.05;
	// predict trajectories
	std::vector<Matrix<double,3,4>> predictedFootsteps;
	predictedFootsteps.resize(N);

	static MatrixXd predLegPhase(N,4);
	predictedFootsteps=predictTrajectories(N, pred_dt, &predLegPhase);
	static MatrixXd scaled_phases(4,N);
	scaled_phases=predLegPhase.transpose();


	//------------------------------- MESSY FOOTSTEP ANGLE PREDICTION -------------------------------
	double footX, footY, tmp;
	for(int i=0; i<N; i++)
	{

		for(int j=0; j<4; j++)
		{	
			if(predLegPhase(i,j)<0)
					predLegPhase(i,j)+=1;

			if(predLegPhase(i, j)<Duty(j/2))
			{
				tmp=(predLegPhase(i,j))/(Duty(j/2));	
				/*if(j<2)
				{
					footX = (1-tmp)*trPointsFL0(0,0) + tmp*trPointsFL0(0,2);
					footY = (1-tmp)*trPointsFL0(1,0) + tmp*trPointsFL0(1,2);
				} 
				else
				{
					footX = (1-tmp)*trPointsHL0(0,0) + tmp*trPointsHL0(0,2);
					footY = (1-tmp)*trPointsHL0(1,0) + tmp*trPointsHL0(1,2);
				}*/
				footX=(1-tmp)*stanceStart(0,0) + tmp*stanceEstEnd(0,0);
				footY=(1-tmp)*stanceStart(1,0) + tmp*stanceEstEnd(1,0);
			}
			else
			{
				
				tmp=(predLegPhase(i,j)-Duty(j/2))/(1-Duty(j/2));	  
				/*if(j<2)
				{
					footX = (1-tmp)*trPointsFL0(0,2) + tmp*trPointsFL0(0,0);
					footY = (1-tmp)*trPointsFL0(1,2) + tmp*trPointsFL0(1,0);
				} 
				else
				{
					footX = (1-tmp)*trPointsHL0(0,2) + tmp*trPointsHL0(0,0);
					footY = (1-tmp)*trPointsHL0(1,2) + tmp*trPointsHL0(1,0);
				}*/
				footX=(1-tmp)*stanceEstEnd(0,0) + tmp*stanceStart(0,0);
				footY=(1-tmp)*stanceEstEnd(1,0) + tmp*stanceStart(1,0);
			}

			scaled_phases(j,i)=atan2(footX, footY);
		}
	}

	scaled_phases.block(0,0,1,N)*=-1;
	scaled_phases.block(2,0,1,N)*=-1;



	// get reference for both girdles
	static MatrixXd girdleRefFromLegs(2,N);
	girdleRefFromLegs.block(0,0,1,N) = spineCPGscaling(0)*(scaled_phases.block(0,0,1,N) + scaled_phases.block(1,0,1,N));
	girdleRefFromLegs.block(1,0,1,N) = spineCPGscaling(1)*(scaled_phases.block(2,0,1,N) + scaled_phases.block(3,0,1,N));



	// init cpg
	double a=1;
	Vector2d R=girdleRefFromLegs.rowwise().maxCoeff();
	static MatrixXd theta=MatrixXd::Zero(2,2), r=MatrixXd::Zero(2,2);
	Vector2d dr, dtheta;



	//----------------------------- DFT PHASE ESTIMATION ------------------------------
	double re, im;
	static MatrixXd phase_est=MatrixXd::Zero(2,2);
	double phase_diff;
	static Vector2d phase_correction;
	for(int i=0; i<2; i++){
		re=0; im=0;
		for(int k=0; k<N; k++){
			re = re + girdleRefFromLegs(i,k)*cos(2*pi*freq_walk*k*pred_dt);
			im = im - girdleRefFromLegs(i,k)*sin(2*pi*freq_walk*k*pred_dt);
		}
		phase_est(i, 0)=phase_est(i, 1);
		phase_est(i, 1)=atan2(im, re);


		phase_correction(i)=phase_est(i,0) + round((theta(i,0) - phase_est(i,0))/(2*my_pi))*2*my_pi   - theta(i,0);
	}
	
	

	//------------------------------------- RUN CPG -------------------------------------

	dtheta = 2*pi*freq_walk*MatrixXd::Ones(2,1) + 2*phase_correction;

	// update radius
	R=R*(1-abs(gamma))*abs(cos(walkingDirection));
	dr = a*(R-r.block(0,0,2,1));

	// euler integration
	theta.block(0,1,2,1)=theta.block(0,0,2,1) + dt*dtheta;
	r.block(0,1,2,1)=r.block(0,0,2,1) + dt*dr;

	// angle output
	girdleCpgOutput(0)=r(0,0)*cos(theta(0,0));
	girdleCpgOutput(1)=r(1,0)*cos(theta(1,0));

	// udpate old values
	theta.block(0,0,2,1)=theta.block(0,1,2,1);
	r.block(0,0,2,1)=r.block(0,1,2,1);





	return;
}


/* Calculate girdle oscillations to follow leg movements */
void
Controller :: FixedGirdleOscillations()
{


	// init cpg
	double a=1;
	Vector2d R=spineCPGscaling;
	static MatrixXd theta=MatrixXd::Zero(2,2), r=MatrixXd::Zero(2,2);
	Vector2d dr, dtheta;

	static bool is_init=false;
	if(!is_init){
		is_init=true;
		theta.block(0,0,2,1) << my_pi, 0;
	}


	//------------------------------------- RUN CPG -------------------------------------


	dtheta = 2*pi*freq_walk*MatrixXd::Ones(2,1);

	// update radius
	R=R*(1-abs(gamma))*abs(cos(walkingDirection));
	dr = a*(R-r.block(0,0,2,1));

	// euler integration
	theta.block(0,1,2,1)=theta.block(0,0,2,1) + dt*dtheta;
	r.block(0,1,2,1)=r.block(0,0,2,1) + dt*dr;

	// angle output
	girdleCpgOutput(0)=r(0,0)*cos(theta(0,0));
	girdleCpgOutput(1)=r(1,0)*cos(theta(1,0));

	// udpate old values
	theta.block(0,0,2,1)=theta.block(0,1,2,1);
	r.block(0,0,2,1)=r.block(0,1,2,1);





	return;
}