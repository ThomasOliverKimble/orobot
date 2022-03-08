 
/* PLEUROBOT UTILITIES */
#ifndef MISC_MATH_CPP
#define MISC_MATH_CPP


#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <sys/time.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>

using namespace Eigen;
using namespace std; 





// filtering
double pt1(double u, double y, double T, double Ts);
MatrixXd pt1_vec(MatrixXd u, MatrixXd y, double T, double Ts);

// basic rotation matrices
Matrix3d rotx(double ang);
Matrix3d roty(double ang);
Matrix3d rotz(double ang);


/* Hermite spline */
MatrixXd hermiteSpline(Vector2d p0, Vector2d p1, Vector2d m0, Vector2d m1, MatrixXd t);


/* Save acos */
double SafeAcos (double x);


/* PID controller */

class PID
{
	public:
		PID(double Kpin, double Kiin, double Kdin, double dt);
		PID(){};
		PID& operator=( const PID& other );
		double calc(double e);
	private:
		double Kp, Ki, Kd, Ts;   // Gains
		double a, b, c;
		double ek1, ek2, uk1; // past values
};


/* PIDvec controller */

class PIDvec
{
	public:
		PIDvec(){};
		void init(MatrixXd Kpin, MatrixXd Kiin, MatrixXd Kdin, MatrixXd Tdin, double dt);
		void init(double Kpin, double Kiin, double Kdin, double Tdin, double dt);
		PIDvec& operator=( const PIDvec& other );
		MatrixXd calc(MatrixXd e);
		MatrixXd calc(double e);
		void flush();
	private:
		MatrixXd Kp, Ki, Kd, Td; // Gains
		double Ts;   
		MatrixXd a, b, c, d, f, g;
		MatrixXd ek1, ek2, uk1, uk2; // past values
};


#endif

