 #include "controller.hpp" 


/* Reads joystick */
void
Controller :: readJoystick()
{
    js.update();

       
    // PS3 CONTROLLER
    joy_sel=js.buttons[0];
    joy_l3=js.buttons[1];
    joy_r3=js.buttons[2];
    joy_start=js.buttons[3];
    joy_l2=js.buttons[8];
    joy_r2=js.buttons[9];
    joy_l1=js.buttons[10];
    joy_r1=js.buttons[11];
    joy_bU=js.buttons[12];
    joy_bR=js.buttons[13];
    joy_bD=js.buttons[14];
    joy_bL=js.buttons[15];

    joy_x1=js.axes[0];
    joy_y1=-js.axes[1];
    joy_x2=js.axes[2];
    joy_y2=-js.axes[3];
    joy_x3=js.axes[4];
    joy_y3=-js.axes[5];
     




    joy_lsr=sqrt(joy_x1*joy_x1+joy_y1*joy_y1);
    joy_lsr=joy_lsr>1?1:joy_lsr;
    if(joy_lsr<0.1){
        joy_lsr=0;
    }
    if(joy_lsr>0.2){
        joy_lsphi=atan2(-joy_x1, joy_y1);
    }
    else{
        joy_lsphi=0;
    }


    joy_rsr=sqrt(joy_x2*joy_x2+joy_y2*joy_y2);
    joy_rsr=joy_rsr>1?1:joy_rsr;
    if(joy_rsr<0.1){
        joy_rsr=0;
    }
    if(joy_rsr>0.2){
        joy_rsphi=atan2(joy_y2, joy_x2)-my_pi/2;
    }
    else{
        joy_rsphi=0;
    }




}

/* Reads joystick inputs and modifies trajectories */
bool
Controller :: updateState()
{
    if(USE_JOYSTICK){
        readJoystick();
    }
    else{
        joy_bU=0;
        joy_bR=0;
        joy_bD=0;
        joy_bL=0;
        joy_l2=0;
        joy_r2=0;
        joy_l1=0;
        joy_r1=0;
        joy_sel=0;
        joy_start=0;
        joy_l3=0;
        joy_r3=0;

        joy_x1=0;
        joy_y1=0;
        joy_x2=0;
        joy_y2=0;
        joy_x3=0;
        joy_y3=0;

        joy_lsr=1;
        joy_rsr=1;
        joy_lsphi=0;
        joy_rsphi=0;
    }


    //============================== EMERGENCY STOP ==============================
    if(joy_start == 1){
        return false;
    }

    //=============================== POSING ===================================
    if(joy_sel==0 && joy_bU==1 && state != POSING){
        state=POSING;
        posing_head=0;
        T_trans=T_trans0;
    }

    //=============================== ANIMAL_WALKING ===================================
    else if(joy_sel==0 && joy_bL==1 && state != ANIMAL_WALKING){
        state=ANIMAL_WALKING;
        T_trans=T_trans0;
    }


    //=============================== STANDING ===================================

    else if(joy_sel==0 && joy_bR==1 && !(state==WALKING || state == STANDING)){
        state=STANDING;
        T_trans=T_trans0;
    }

    //=============================== WALKING ===================================
    if(((state == STANDING && ((joy_lsr>0.2))) || !USE_JOYSTICK)){
        state=WALKING;
    }

    //=============================== STANDING ===================================
    else if(state == WALKING && (!(joy_lsr>0.2)) && (legs_stance(0) && legs_stance(1) && legs_stance(2) && legs_stance(3))){
        state=STANDING;
    }

    //=============================== INITIAL ===================================
    if(joy_sel==0 && joy_bD==1 && !(state==INITIAL)){
        state=INITIAL;
        T_trans=T_trans0;
    }





    //=============================== DECAY TRANSITION FILTERING CONSTANT ===================================
    T_trans=pt1(0, T_trans, 1, dt);


    //=============================== JOYSTICK INTERACTION ===================================
    if(AUTO_RUN){
        joy_lsr=1;
        state=AUTO_RUN_STATE;
    }

    joystickManipulation();
    return true;
}

/* Joystick manipulation of different parameters/variables */
void
Controller :: joystickManipulation()
{
    if(state==INITIAL){
        angles=init_angles;
    }

}

