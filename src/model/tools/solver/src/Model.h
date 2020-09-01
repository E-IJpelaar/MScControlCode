#ifndef MODEL_H
#define MODEL_H

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <unistd.h>
#include "liegroup.h"
// #include "shapesx.cpp"


#ifdef DISCONTINIOUS
#include "shapesx.cpp"
#else
#include "shapes.cpp"
#endif


#include "bin/tictoc.h"
#include "bin/smoothstep.h"
#include "Config/ConfigFile.cpp"
#include "Config/Chameleon.cpp"

#define SOLVER_OUTPUT
#define TICTOC
//#define EIGEN_NO_DEBUG
//#define JACOBIAN

#define PRECISION 5
#define PI 3.1415926

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

typedef Eigen::Matrix<float, 19, 1> Vff;         // Vff = 19x1 vector with floats

class Model
{
  public:                                        // public variables

  	bool ENERGY_CONTROLLER;
  	bool WRITE_OUTPUT;

  	int E1,E2,E3;
  	int K1,K2,K3; 

  	float E11, E22, E33;
  	float G11, G22, G33;

  	int NMODE; 
  	int NDISC;
	float SDOMAIN; 
	float TDOMAIN; 
	int SPACESTEP;
	float TIMESTEP; 
	int INTSTEP;

	float ATOL;
	float RTOL; 
	int MAX_IMPL;
	int MAX_ITER; 
	float SPEEDUP; 

	float PRS_AREA; 
	float GRAVITY; 
	float RADIUS; 
	float RHO; 
	float EMOD; 
	float NU; 
	float MU; 

  	float KP, KD;
  	int NDof, NState;
  	Shapes Phi;                                        // class "Shapes" is made in "shapes.h"
	  

  	Mxf Hess;

  	Mxf Ba,Bc;                                         //  here the different variables get there matrix/vector size defined
  	Mxf Sa,Sc;
  	M6f Mtt, Ktt, Dtt;
  	Mxf Mtee, Mee, Cee, Kee, Dee;
  	Vxf Gee;
  	Vxf q, dq, ddq;
  	Vxf Qa, Qv, Qu, Qd;
  	Vxf qd;
  	V6f Xi0;

  	V7f g;
  	V6f xi,dxi,ddxi,eta,deta,lam;
	V4f quat;
	V3f xiK, xiE;
	M3f R; 
	M4f A;

	V6f gvec;
	Vxf z0, tau, u;

	Model(const char* str);

	void output(ofstream &file, float t, Vxf x);
	void read(const char* str, Vxf &x);
	void cleanup();

	void controllerPassive(float t, Mxf &M, Mxf &C,	Vxf &G, Vxf &f);

	Vxf implicit_simulate();

	void buildJacobian(float se, Mxf &J, Mxf &Jt);
	void buildLagrange(Vxf v, Vxf dv, Mxf &M, Mxf &C, Vxf &dG, Mxf &Mt);
	
	void dynamicODE(float t, Vxf x, Vxf &dx);
	void jacobiODE(float s, V13f x, V13f &dx, Mxf &dJ, Mxf &dJt);
	void systemMatODE(float s,Mxf &K, Mxf &M, Mxf &D);
	void lagrangianODE(float s, V13f x, Mxf J, Mxf Jt,
	V13f &dx, Mxf &dJ, Mxf &dJt, Mxf &dM, Mxf &dC, Vxf &dG, Mxf &dMt);

	Mxf tableConstraints(Vxi table, bool set = true);
	void hessianInverse(float dt, Vxf R, Vxf &dx);

	void buildInertiaTensor();
	void buildStiffnessTensor();
	void buildDampingTensor();
	void buildGlobalSystem();

	Mxf pressureMapping();
};

#endif