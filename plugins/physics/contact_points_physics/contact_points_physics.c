/*
 * Description:  Example of use of a physics plugin to get information on
 *               the contact points and ground reaction forces of an object
 *               that collides with the floor
 */

#include <plugins/physics.h>
#include <assert.h>
#define MAX_CONTACTS 10

static pthread_mutex_t mutex;

// plugin variables
static dGeomID floor_geom = NULL;
static dGeomID footGeom[4][2];
static dBodyID footBody[4][2];
static dContact contacts[4][2][MAX_CONTACTS];
static dJointID contact_joints[4][2][MAX_CONTACTS];
static dJointFeedback feedbacks[4][2][MAX_CONTACTS];
static int nContacts[4][2] = {{0}};
static double finalForce[12] = {0};
// plugin function called by Webots at the beginning of the simulation
void webots_physics_init() {
  pthread_mutex_init(&mutex, NULL); // needed to run with multi-threaded version of ODE

  // get ODE handles to .wbt objects
  floor_geom = dWebotsGetGeomFromDEF("FLOOR");

  footGeom[0][0] = dWebotsGetGeomFromDEF("foot_fl.SEG1");
  footGeom[0][1] = dWebotsGetGeomFromDEF("foot_fl.SEG2");
  footGeom[1][0] = dWebotsGetGeomFromDEF("foot_fr.SEG1");
  footGeom[1][1] = dWebotsGetGeomFromDEF("foot_fr.SEG2");
  footGeom[2][0] = dWebotsGetGeomFromDEF("foot_hl.SEG1");
  footGeom[2][1] = dWebotsGetGeomFromDEF("foot_hl.SEG2");
  footGeom[3][0] = dWebotsGetGeomFromDEF("foot_hr.SEG1");
  footGeom[3][1] = dWebotsGetGeomFromDEF("foot_hr.SEG2");

  footBody[0][0] = dWebotsGetBodyFromDEF("foot_fl.SEG1");
  footBody[0][1] = dWebotsGetBodyFromDEF("foot_fl.SEG2");
  footBody[1][0] = dWebotsGetBodyFromDEF("foot_fr.SEG1");
  footBody[1][1] = dWebotsGetBodyFromDEF("foot_fr.SEG2");
  footBody[2][0] = dWebotsGetBodyFromDEF("foot_hl.SEG1");
  footBody[2][1] = dWebotsGetBodyFromDEF("foot_hl.SEG2");
  footBody[3][0] = dWebotsGetBodyFromDEF("foot_hr.SEG1");
  footBody[3][1] = dWebotsGetBodyFromDEF("foot_hr.SEG2");



  dWebotsConsolePrintf("GRF PHYSICS PLUGIN ACTIVE\n");


}

// plugin function called by Webots for every WorldInfo.basicTimeStep
void webots_physics_step() {
  int foot_i, seg_i;
  for(foot_i=0; foot_i<4; foot_i++){
    for(seg_i=0; seg_i<2; seg_i++){
      nContacts[foot_i][seg_i] = 0;
    }
  }
}

// it is not really necessary but it's fun to draw the contact points
/*void webots_physics_draw(int pass, const char *view) {
  if (pass != 1)
    return;

  // change OpenGL state
  glDisable(GL_LIGHTING);    // not necessary
  glLineWidth(10);           // use a thick line
  glDisable(GL_DEPTH_TEST);  // draw in front of Webots graphics

  int i;
  int foot_i, seg_i;
  for(foot_i=0; foot_i<4; foot_i++){
    for(seg_i=0; seg_i<2; seg_i++){
      for (i = 0; i < nContacts[foot_i][seg_i]; i++) {
        dReal *p = contacts[foot_i][seg_i][i].geom.pos;
        dReal *n = contacts[foot_i][seg_i][i].geom.normal;
        dReal d = contacts[foot_i][seg_i][i].geom.depth * 300;
        glBegin(GL_LINES);
        glColor3f(1, 0, 0);  // draw in red
        glVertex3f(p[0], p[1], p[2]);
        glVertex3f(p[0] + n[0] * d, p[1] + n[1] * d, p[2] + n[2] * d);
        glEnd();
      }
    }
  }
  
}*/

// This function is implemented to overide Webots collision detection.
// It returns 1 if a specific collision is handled, and 0 otherwise.
int webots_physics_collide(dGeomID g1, dGeomID g2) {
  
  int foot_i, seg_i;
  for(foot_i=0; foot_i<4; foot_i++){
    for(seg_i=0; seg_i<2; seg_i++){

        // check if this collision involves the objects which interest us
        if ((dAreGeomsSame(g1, footGeom[foot_i][seg_i]) && dAreGeomsSame(g2, floor_geom)) || (dAreGeomsSame(g2, footGeom[foot_i][seg_i]) && dAreGeomsSame(g1, floor_geom))) {
            dBodyID body = dGeomGetBody(g1);
            if (body==NULL) body = dGeomGetBody(g2);
            if (body==NULL) return 0;
            dWorldID world = dBodyGetWorld(body);
            dJointGroupID contact_joint_group = dWebotsGetContactJointGroup();
            // see how many collision points there are between these objects
            nContacts[foot_i][seg_i] = dCollide(g1, g2, MAX_CONTACTS, &contacts[foot_i][seg_i][0].geom, sizeof(dContact));
            int i;
            for (i = 0; i < nContacts[foot_i][seg_i]; i++) {
              // custom parameters for creating the contact joint
              // remove or tune these contact parameters to suit your needs
              contacts[foot_i][seg_i][i].surface.mode = dContactBounce | dContactSoftCFM | dContactApprox1;
              contacts[foot_i][seg_i][i].surface.mu = 0.4;
              contacts[foot_i][seg_i][i].surface.bounce = 0;
              contacts[foot_i][seg_i][i].surface.bounce_vel = 0;
              contacts[foot_i][seg_i][i].surface.soft_erp = 0.2;
              contacts[foot_i][seg_i][i].surface.soft_cfm = 0.001;

              // create a contact joint that will prevent the two bodies from intersecting
              // note that contact joints are added to the contact_joint_group
              pthread_mutex_lock(&mutex);
              contact_joints[foot_i][seg_i][i] = dJointCreateContact(world, contact_joint_group, &contacts[foot_i][seg_i][i]);

              // attach joint between the body and the static environment (0)
              dJointAttach(contact_joints[foot_i][seg_i][i], footBody[foot_i][seg_i], 0);

              // attach feedback structure to measure the force on the contact joint
              dJointSetFeedback(contact_joints[foot_i][seg_i][i], &feedbacks[foot_i][seg_i][i]);
              pthread_mutex_unlock(&mutex);
            }

            return 1;  // collision was handled above
          }
      } 
    }

  return 0;  // collision must be handled by webots
}

// convenience function to print a 3d vector
static void print_vec3(const char *msg, const dVector3 v) {
  dWebotsConsolePrintf("%s: %g %g %g\n", msg, v[0], v[1], v[2]);
}

// this function is called by Webots after dWorldStep()
void webots_physics_step_end() {
  /*if (nContacts == 0) {
    dWebotsConsolePrintf("no contact\n");
    return;
    }*/

  /*int foot_i, seg_i, i;
  for(foot_i=0; foot_i<4; foot_i++){
    for(seg_i=0; seg_i<2; seg_i++){
      for (i = 0; i < nContacts[foot_i][seg_i]; i++) {
        // printf force and torque that each contact joint
        // applies on the box's body
        dWebotsConsolePrintf("contact: %d:\n", i);
        print_vec3("f1", feedbacks[foot_i][seg_i][i].f1);
        print_vec3("t1", feedbacks[foot_i][seg_i][i].t1);
        print_vec3("f2", feedbacks[foot_i][seg_i][i].f2);
        print_vec3("t2", feedbacks[foot_i][seg_i][i].t2);
        dWebotsConsolePrintf("\n");
      }
    }
  }*/

  // ====================== calculate the resulting force per foot ==================================
  int foot_i, seg_i, i;
  for(foot_i=0; foot_i<4; foot_i++){
    finalForce[foot_i*3  ]=0;
    finalForce[foot_i*3+1]=0;
    finalForce[foot_i*3+2]=0;


    for(seg_i=0; seg_i<2; seg_i++){
      for (i = 0; i < nContacts[foot_i][seg_i]; i++) {
        finalForce[foot_i*3  ] += (double)feedbacks[foot_i][seg_i][i].f1[0];
        finalForce[foot_i*3+1] += (double)feedbacks[foot_i][seg_i][i].f1[1];
        finalForce[foot_i*3+2] += (double)feedbacks[foot_i][seg_i][i].f1[2];
      }
    }
  }


  dWebotsSend(0, finalForce, sizeof(finalForce));

  
}

// this function is called by Webots to cleanup memory before unloading the physics plugin
void webots_physics_cleanup() {
  pthread_mutex_destroy(&mutex);
}
