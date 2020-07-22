#ifndef MODEL_H
#define MODEL_H

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <unistd.h>
#include "liegroup.h"
#include "shapesx.cpp"
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

typedef Eigen::Array<int, Dynamic, 1> Vxi;
typedef Eigen::Array<int, 6, 1> V6i;
typedef Eigen::Matrix<float, 6, 6> M6f;
typedef Eigen::Matrix<float, 4, 4> M4f;
typedef Eigen::Matrix<float, 3, 3> M3f;
typedef Eigen::Matrix<float, 7, 1> V7f;
typedef Eigen::Matrix<float, 13, 1> V13f;
typedef Eigen::Matrix<float, 6, 1> V6f;
typedef Eigen::Matrix<float, 4, 1> V4f;
typedef Eigen::Matrix<float, 3, 1> V3f;
typedef Eigen::VectorXf Vxf;
typedef Eigen::MatrixXf Mxf;
typedef Eigen::VectorXd Vxd;
typedef Eigen::MatrixXd Mxd;

typedef Eigen::Matrix<float, 19, 1> Vff;

class Model
{
  public:

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
  	Shapes Phi;

  	Mxf Hess;

  	Mxf Ba,Bc;
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

	void controllerPassive(float t, Mxf &M, Mxf &C,
	Vxf &G, Vxf &f);

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