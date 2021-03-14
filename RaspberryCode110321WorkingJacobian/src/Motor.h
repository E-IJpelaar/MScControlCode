#ifndef MOTOR_H
#define MOTOR_H

#include <cstdio>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <unistd.h>
#include <termios.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>


#include "pca9685.cpp"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>


#include "Timer.h"

#include "liegroup.h" 
#include "shapes.cpp"


#include "Config/ConfigFile.cpp"
#include "Config/Chameleon.cpp"


#define TICTOC
#define PRECISION 5

#define BUFSIZE 10

#define NMode 1
#define NDof 2
#define NState 2


	typedef Eigen::Array<int, Dynamic, 1> Vxi;       // Dynamic vector with integers
	typedef Eigen::Array<int, 6, 1> V6i;             // V6i = 6x1 vector with integers
	typedef Eigen::Matrix<float, 6, 6> M6f;          // M6f = 6x6 matrix with floats
	typedef Eigen::Matrix<float, 4, 4> M4f;          // M4f = 4x4 matrix with floats
	typedef Eigen::Matrix<float, 3, 3> M3f;          // M3f = 3x3 matrix with floats
	typedef Eigen::Matrix<float, 7, 1> V7f;          // V7f = 7x1 vector with floats	
	typedef Eigen::Matrix<float, 13, 1> V13f;        // V13f = 13x1 vector with floats
	typedef Eigen::Matrix<float, 6, 1> V6f;          // V6f = 6x1 vector with floats         
	typedef Eigen::Matrix<float, 4, 1> V4f;          // V4f = 4x1 vector with floats       
	typedef Eigen::Matrix<float, 3, 1> V3f;          // V3f = 3x1 vector with floats       
	typedef Eigen::VectorXf Vxf;                     // Vxf = nx1 vector with floats
	typedef Eigen::MatrixXf Mxf;                     // Mxf = nxm matrix with floats
	typedef Eigen::VectorXd Vxd;                     // Dynamic vector with doubles
	typedef Eigen::MatrixXd Mxd;                     // Dynamic matrix with doubles

	 




struct DEVICE{
	int INA = -1;
	int INB = -1;
	int EN = -1;
	float KP = 0.0;
};

class Motor
{
  public:
	
	bool MOTOR_ENABLE;
	bool WRITE_OUTPUT;
	char PCA9685_ID;
	char MPU6050_ID;
	char MPRLS25_ID;
	int UPDATE_RATE_HIGHLEVEL;
	int fd;
	int fd_pca;
	
	int PIN_BASE,PWM_FREQ,PWM_MAX;
	int LED;
	
	DEVICE M1, M2, M3, M4;
	float tx,ty,tz;
	
	float KP,KD;
	float t  = 0.0;
	float dt = 0.0;

	float f;
    char *endptr;

	//char buf[BUFSIZE];
	double angle0;
	double pressure,a;
	double p_ref,u;
	double Kp,Ki,Kd,pwm_offset;
	double old_pressure = 0;
	double old_a = 0;
	double old_X = 0;
	double old_Y = 0;
	
	double integral = 0;
	double derivative = 0;
	double pre_error = 0;
	
	int SPACESTEP = 11;
	
	Shapes Phi;
	
	
	
	Mxf Ba;
	Vxf q,dq;
	V6f Xi0;
	//Mxf J;
	
	V7f g;
	V6f xi,dxi,eta;
	V4f quat;
	V3f xiK,xiE;
	M3f R;
	M4f A;

	
	
	

	




	Motor(const char* str);
	




	void setMotor(int id, float pwm);
	double readMPU();
	double readPressure(); // origanlly pressure from arduino is a float
	void reset();
	double PIDcontroller(double setpoint, double current_value, double I_max);
	double MPUcallibration();
	int serialOpenB (const char *device, const int baud, const int max);
	double PressureCallibration();
	double getX();
	double getY();
	
	void buildJacobian(float se, Mxf &J, Mxf &Jt);
	void jacobiODE(float s, V13f x, V13f &dx, Mxf &dJ, Mxf &dJt);
	
	void tic();
	float toc();
	void updateClock();

};

#endif
