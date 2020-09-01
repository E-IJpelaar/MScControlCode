#include "Model.h"

using namespace std;
using namespace Eigen;

ofstream statelog;
ofstream taulog;
ofstream glog;

//---------------------------------------------------
//-------------------------------- class constructor
//---------------------------------------------------
Model::Model(const char* str){

	V6i tab;																		// type V6i  = array, tab = table with 6 integers (Model.h)

	ConfigFile cf(str);                                                             // ?reads in configfile? 

	// static_cast<bool> converts energy_controller to a boolean type
	
	ENERGY_CONTROLLER = static_cast<bool>(cf.Value("options","ENERGY_CONTROLLER"));  // check if energy controller is turned on in config file
	WRITE_OUTPUT = static_cast<bool>(cf.Value("options","WRITE_OUTPUT"));            // check if output is desired in config file


	// static_cast<int> converst possible float to an integer type
 
	K1 = static_cast<int>(cf.Value("cosserat","K1"));	// e_xy   					// curvature-torsion strain   // booleans
	K2 = static_cast<int>(cf.Value("cosserat","K2"));	// e_xz						// curvature-torsion strain
	K3 = static_cast<int>(cf.Value("cosserat","K3"));	// e_yz						// curvature-torsion strain
	E1 = static_cast<int>(cf.Value("cosserat","E1"));	// e_xx						// stretch-shear strain
	E2 = static_cast<int>(cf.Value("cosserat","E2"));	// e_yy						// stretch-shear strain
	E3 = static_cast<int>(cf.Value("cosserat","E3"));	// e_zz						// stretch-shear strain

	tab << K1,K2,K3,E1,E2,E3;                                                       // put K1 to E3 in "tab"

	NDISC   = static_cast<int>(cf.Value("model","NDISC"));                          // assign NDISC from config.txt under section "model"
	NMODE   = static_cast<int>(cf.Value("model","NMODE"));							// assign NMMODE from config.txt under section "model"
  	SDOMAIN = cf.Value("model","SDOMAIN");                                          // length of robot
  	TDOMAIN = cf.Value("model","TDOMAIN");                                          // time domain from config file

  	SPACESTEP = static_cast<int>(cf.Value("solver","SPACESTEP"));                   // discretize the length of the robot
  	INTSTEP   = static_cast<int>(cf.Value("solver","INTSTEP"));                     // integral stepsize for mass/stifness matrix
  	MAX_IMPL  = static_cast<int>(cf.Value("solver","MAX_IMPL"));                    // maximum amount of implicit time steps
  	MAX_ITER  = static_cast<int>(cf.Value("solver","MAX_ITER"));                    // maximum amount of solver iteration 
  	ATOL      = cf.Value("solver","ATOL");                                          // absolute tolerance
  	RTOL      = cf.Value("solver","RTOL");											// relative tolerance
  	SPEEDUP   = cf.Value("solver","SPEEDUP");                                       // used in static solver
  	TIMESTEP  = cf.Value("solver","TIMESTEP");                                      // discretize time

  	RHO      = cf.Value("physics","RHO");                                           // mass density
  	EMOD     = cf.Value("physics","EMOD");                                          // E modulus material
  	NU       = cf.Value("physics","NU");											// poison ratio
  	MU       = cf.Value("physics","MU"); 											// damping coefficeint
  	PRS_AREA = cf.Value("physics","PRS_AREA");										// effective pressure area 
  	GRAVITY  = cf.Value("physics","GRAVITY");                                       // gravitional acceleration
  	RADIUS   = cf.Value("physics","RADIUS");                                        // radius of robot 

  	KP = cf.Value("control","KP");                                                  // controller proportional gain
  	KD = cf.Value("control","KD");                                                  // controller derivative gain

	Ba = tableConstraints(tab,true);												// CHECK HOW FUNCTION WORKS ? active DOF ?
	Bc = tableConstraints(tab,false);										        // contraint DOF
	NDof = Ba.cols();																// amount of active DOF. equal to collums of Ba
	NState = NDof * NMODE;															// ? amount of states ?


	Vxi sa(NMODE/NDISC), sc(NMODE), stab(NState);
    stab.setZero();
    stab(0) = 1;

 

    #ifdef DISCONTINIOUS
        sa.setZero();
        sa(0) = 1;
        sc = sa.replicate(NDISC,1);
        Phi.set(NMODE,NDof,NDISC,"legendre");
        cout << "DISCONTINIOUS = true" << endl;
    #else
        sc.setZero();
        sc(0) = 1.0;
        Phi.set(NMODE,NDof,"legendre");
        cout << "CONTINIOUS = true" << endl;
    #endif

 

    stab = sc.replicate(NDof,1);
    Sa = tableConstraints(stab,true);
    Sc = tableConstraints(stab,false);

 

    Sa.transposeInPlace();
    Sc.transposeInPlace();
	

	// build Inertia Tensor, Stiffness Tensor, Damping Tensor and Global system as called in functions
	// check how each function works
	
	buildInertiaTensor();													         
	buildStiffnessTensor();
	buildDampingTensor();
	buildGlobalSystem();



	q   = Vxf::Constant(NState,ATOL);    // ? create constant vector ? Why these values especially ATOL ?
	dq  = Vxf::Zero(NState);             // create zero vectors
	ddq = Vxf::Zero(NState);
	qd  = Vxf::Zero(NState);			// ? Vxf is undifined length matrix??
	tau = Vxf::Zero(NDof*NDISC);        // control input vector
	u   = Vxf::Zero(6);
	z0  = Vxf::Zero(6);

	Xi0 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
	gvec << 0.0, 0.0, 0.0, GRAVITY, 0.0, 0.0;    //  ? why is gravity 4th entry? rot1 rot2 rot3 x y z ?


	// read input
	read("state.log",q);               // read state.log = initial position
	//read("state_dt.log",dq);         // ? read state_dt.log = initial velocities ? 
	//read("input.log",tau);           // read control input tau
	read("point.log",qd);              // read point.log = desired position

	// clean up 
	cleanup();                          // cleans output files

	statelog.open("state.log", ios_base::app | ios::binary);    // 
	taulog.open("tau.log", ios_base::app | ios::binary);
	glog.open("g.log", ios_base::app | ios::binary);

	cout << setprecision(PRECISION) << endl;
	//q(0) = 1;
}	

//---------------------------------------------------
//-------------------------------------- output file
//---------------------------------------------------
void Model::output(ofstream &file, float t, Vxf x){

	  file << t;

	  for (int i = 0; i < x.size(); ++i)
	  {
	  	file << ", " << x(i);
	  }

	  file << "\n";
}

//---------------------------------------------------
//---------------------------------------- read file
//---------------------------------------------------
void Model::read(const char* str, Vxf &x){

  char data[100];
  ifstream infile;
  infile.open(str);

  for (int i = 0; i < x.size(); ++i)
  {
  	infile >> data;
  	x(i) = (float)atof(data);
  }
}

//---------------------------------------------------
//--------------------------------------- clean file
//---------------------------------------------------
void Model::cleanup(){
	ofstream myfile1, myfile2, myfile3;
	myfile1.open("state.log", ofstream::out | ofstream::trunc);
	myfile2.open("tau.log", ofstream::out | ofstream::trunc);
	myfile3.open("g.log", ofstream::out | ofstream::trunc);
	myfile1.close();
	myfile2.close();
	myfile3.close();
}

//---------------------------------------------------
//-- passivity-based controller for end-effector
//---------------------------------------------------
void Model::controllerPassive(float t, Mxf &M, Mxf &C,
Vxf &G, Vxf &f){

	int na, nc;
	na = NDof*NDISC;
	nc = NState - na;

	Mxf S(NState,NState);
	Mxf M11(na,na), M12(na,nc);
	Mxf M21(nc,na), M22(nc,nc); 
	Mxf Ms(NState,NState);
	Mxf M22_(na,na);
	Vxf Hs(NState);
	Vxf H1(nc), H2(na);
	Vxf H2_(na);

	S.block(0,0,na,NState).noalias()  = Sc;
	S.block(na,0,nc,NState).noalias() = Sa;

	// partition mass matrix;
	Ms.noalias() = S*M*S.transpose();
	M11.noalias() = Ms.block(0,0,nc,nc);
	M12.noalias() = Ms.block(0,nc,nc,na);
	M21.noalias() = Ms.block(nc,0,na,nc);
	M22.noalias() = Ms.block(nc,nc,na,na);

	// partition non-inertial forces
	Hs.noalias() = S*(C*dq + G + 0*Kee*q);
	H1.noalias() = Hs.block(0,0,nc,1);
	H2.noalias() = Hs.block(nc,0,na,1);

	M22_.noalias() = M22 - M21*M11.householderQr().solve(M12);
	H2_.noalias() = H2 - M21*M11.householderQr().solve(H1);

	f.noalias() = M22_*(-KP*Sa*(q-qd) - KD*Sa*dq) + H2_; 
}

//--------------------------------------------------
//------------------ implicit solve dynamic problem
//--------------------------------------------------
Vxf Model::implicit_simulate(){
	int n = NState;
	int i = 0;
	int j = 0;
	float t = 0; 
	float dt = (1.0*(TIMESTEP));

	Vxf K1(2*n),K2(2*n);
	Vxf x(2*n),dx(2*n),R(2*n);
	Vxf dr(2*n);

	M6f tmp;

	x.block(0,0,n,1) = q;
	x.block(n,0,n,1) = dq;

	#ifdef TICTOC
		tic();
	#endif

	// solve implicit time integration
	while (t < TDOMAIN && i < MAX_ITER){

		dynamicODE(t,x,K1);

		R.noalias() = -dt*K1;
		hessianInverse(dt,R,dr);
		dx.noalias() = -dr;

		#ifndef QUASINETWON
		while(abs(R.block(n,0,n,1).norm()) > RTOL && j < MAX_IMPL){
			dynamicODE(t+dt,x+dx,K2);
			R.noalias() = dx - 0.5*dt*(K1 + K2);
			hessianInverse(dt,R,dr);
			dx.noalias() -= dr;
			j++;
		}
		#endif

  		if(isnan(dx.norm())){
  			i = MAX_ITER; 
  		}

  		if(WRITE_OUTPUT){
  			admap(eta,tmp);

			output(statelog,t,x.block(0,0,n,1));
			output(taulog,t,tmp*xi);
			output(glog,t,g);
		}

  		x.noalias() += dx;
  		t += dt;
  		i++;
  		j = 0.0;
	}

	#ifdef TICTOC
		toc((float)TDOMAIN);
	#endif

	// return solutions q* = q(t_eq)
	return x.block(0,0,n,1);
}

//---------------------------------------------------
//-------------------------------------- dynamic ode
//---------------------------------------------------
void Model::dynamicODE(float t, Vxf x, Vxf &dx){
	
	int n = NState;
	Vxf x1(n), x2(n);
	Vxf Q(n), Qa(n), Qv(n), Qu(n);

	Qa.setZero();
	Qv.setZero();
	Qu.setZero();

	// extract the generalized coordinates 
	x1.noalias() = x.block(0,0,n,1);
	x2.noalias() = x.block(n,0,n,1);	

	// compute Lagrangian model
	buildLagrange(x1,x2,Mee,Cee,Gee,Mtee);
	Qv.noalias() += Cee*x2;
	Qa.noalias() += Gee;

	// add elastic material forces
	Qa.noalias() += Kee*x1;

	// add viscous material forces
	Qv.noalias() += Dee*x2;

	if(ENERGY_CONTROLLER){
		controllerPassive(t,Mee,Cee,Gee,tau);

		Qu.noalias() = Sa.transpose()*tau;
	}

	(dx.block(0,0,n,1)).noalias() = x2;
	(dx.block(n,0,n,1)).noalias() = Mee.llt().solve(-Qa - Qv + Qu);
}

//---------------------------------------------------
//---------------------------- build Jacobian matrix
//---------------------------------------------------
void Model::buildJacobian(float se, Mxf &J, Mxf &Jt){

	int n = NState;
	double ds,s;

	Mxf K1J(6,n), K2J(6,n);
	Mxf K1Jt(6,n), K2Jt(6,n);
	M6f adeta, AdgInv;
	V13f K1, K2;
	V13f x, dx;

	// initial Jacobian matrix
	J.setZero();
	Jt.setZero();
	x.setZero();
	x(0) = 1.0;

	// compute forward integration step
	ds = (1.0*(se))/(1.0*(SPACESTEP));
	s = 0.0;

	// do spatial integration
	for (int i = 0; i < SPACESTEP; i++){
		jacobiODE(s,x,K1,K1J,K1Jt);
 		jacobiODE(s+(2.0/3.0)*ds, x+(2.0/3.0)*ds*K1, K2, K2J, K2Jt);

  		s += ds;
  		x.noalias() += (ds/4.0)*(K1+3.0*K2);
  		J.noalias() += (ds/4.0)*(K1J+3.0*K2J);
  		Jt.noalias() += (ds/4.0)*(K1Jt+3.0*K2Jt);
	}

	// return configuration and velocities
	g.noalias()   = x.block(0,0,7,1);
	eta.noalias() = x.block(7,0,6,1);

	// compute adjoint actions
	AdmapInv(g,AdgInv);
	admap(eta,adeta);

	// transform Jacobian to local frame
	K1J.noalias() = AdgInv*J;
	J.noalias() = K1J;

	// transform time-derivative Jacobian to local frame
	K1Jt.noalias() = AdgInv*Jt;
	Jt.noalias()  = K1Jt;
}

//---------------------------------------------------
//--------------------------  build lagrangian model
//---------------------------------------------------
void Model::buildLagrange(Vxf v, Vxf dv, 
	Mxf &M, Mxf &C, Vxf &G, Mxf &Mt){

	int n = NState;
	double ds,s;

	Mxf J(6,n), K1J(6,n), K2J(6,n);
	Mxf Jt(6,n), K1Jt(6,n), K2Jt(6,n);
	Mxf K1M(n,n), K2M(n,n);
	Mxf K1Mt(n,n), K2Mt(n,n);
	Mxf K1C(n,n), K2C(n,n);
	Vxf K1G(n), K2G(n);
	V13f K1, K2;
	V13f x, dx;

	// initial matrices
	J.setZero();
	Jt.setZero();
	M.setZero();
	Mt.setZero();
	C.setZero();
	G.setZero();
	x.setZero();
	x(0) = 1.0;

	// set states
	q.noalias() = v;
	dq.noalias() = dv;

	// compute forward integration step
	ds = (1.0*(SDOMAIN))/(1.0*(SPACESTEP));
	s = 0.0;

	// do spatial integration
	for (int i = 0; i < SPACESTEP; i++){

  		lagrangianODE(s,x,J,Jt,K1,K1J,K1Jt,K1M,K1C,K1G,K1Mt);
 		lagrangianODE(s+(2.0/3.0)*ds,
 					  x+(2.0/3.0)*ds*K1, 
 				 	  J+(2.0/3.0)*ds*K1J,
 					  Jt+(2.0/3.0)*ds*K1Jt,
 				      K2,K2J,K2Jt,K2M,K2C,K2G,K2Mt);

 		s += ds;
  		x.noalias()  += (ds/4.0)*(K1+3.0*K2);  
  		J.noalias()  += (ds/4.0)*(K1J+3.0*K2J);
  		Jt.noalias() += (ds/4.0)*(K1Jt+3.0*K2Jt);
  		M.noalias()  += (ds/4.0)*(K1M+3.0*K2M);
  		C.noalias()  += (ds/4.0)*(K1C+3.0*K2C);
  		G.noalias()  += (ds/4.0)*(K1G+3.0*K2G);
  		Mt.noalias()  += (ds/4.0)*(K1Mt+3.0*K2Mt);
	}
}

//---------------------------------------------------
//--------------- forward integrate compute Jacobian
//---------------------------------------------------
void Model::jacobiODE(float s, V13f x, V13f &dx, 
	Mxf &dJ, Mxf &dJt){

	Mxf PMat(NDof,NState);
	M6f adxi, Adg, adeta;

	// evaluate strain-field
	Phi.eval(s,PMat);
	xi.noalias()  = (Ba*PMat)*q + Xi0;
	dxi.noalias()  = (Ba*PMat)*dq;

	// decomposition configuration space
	quat.noalias() = x.block(0,0,4,1);
	eta.noalias()  = x.block(7,0,6,1);
	xiK.noalias()  = xi.block(0,0,3,1);
	xiE.noalias()  = xi.block(3,0,3,1);

	// precompute adjoint actions
	Admap(x.block(0,0,7,1),Adg);
	admap(xi,adxi);
	admap(eta,adeta);

	quat2rot(quat,R);
	strainMapping(R*xiK,A);

	(dx.block(0,0,4,1)).noalias() = (1.0/(2*quat.norm()))*A*quat;
	(dx.block(4,0,3,1)).noalias() = R*xiE;
	(dx.block(7,0,6,1)).noalias() = -adxi*eta + dxi;

	dJ.noalias() = Adg*(Ba*PMat);
	dJt.noalias() = Adg*adeta*(Ba*PMat);
}

//---------------------------------------------------
//--------------- forward integrate Lagrangian model
//---------------------------------------------------
void Model::lagrangianODE(float s, V13f x, Mxf J, Mxf Jt,
	V13f &dx, Mxf &dJ, Mxf &dJt, Mxf &dM, Mxf &dC, Vxf &dG,
	Mxf &dMt){

	Mxf PMat(NDof, NState);
	M6f Adg, Adg_;
	M6f adxi, adeta;

	// evaluate strain-field
	Phi.eval(s,PMat);
	xi.noalias()   = (Ba*PMat)*q + Xi0;
	dxi.noalias()  = (Ba*PMat)*dq;

	// decomposition configuration space
	quat.noalias() = x.block(0,0,4,1);
	eta.noalias()  = x.block(7,0,6,1);
	xiK.noalias()  = xi.block(0,0,3,1);
	xiE.noalias()  = xi.block(3,0,3,1);

	quat2rot(quat,R);
	strainMapping(R*xiK,A);

	// precompute adjoint actions
	Admap(x.block(0,0,7,1),Adg);
	AdmapInv(x.block(0,0,7,1),Adg_);
	admap(xi,adxi);
	admap(eta,adeta);

	// compute local D-configuration
	(dx.block(0,0,4,1)).noalias() = (1.0/(2*quat.norm()))*A*quat;
	(dx.block(4,0,3,1)).noalias() = R*xiE;
	(dx.block(7,0,6,1)).noalias() = -adxi*eta + dxi;

	// compute local D-Jacobian
	dJ.noalias()  = Adg*(Ba*PMat);
	dJt.noalias() = Adg*adeta*(Ba*PMat);

	// compute local D-inertia matrix
	dM.noalias() = (Adg_*J).transpose()*Mtt*(Adg_*J);

	// compute local C-coriolis matrix
	dC.noalias() = (Adg_*J).transpose()*((Mtt*adeta - 
	 	adeta.transpose()*Mtt)*(Adg_*J) + Mtt*(Adg_*Jt));

	// compute local G-potential vector
	dG.noalias() = (Adg_*J).transpose()*(Adg).transpose()*Mtt*gvec;

	// compute local time-derivative of G-inertia matrix
	dMt.noalias() = (Adg_*Jt).transpose()*Mtt*(Adg_*J) + (Adg_*J).transpose()*Mtt*(Adg_*Jt);

}

//---------------------------------------------------
//----------- convert table to active/constraint set
//---------------------------------------------------
Mxf Model::tableConstraints(Vxi table, bool set){
	int k = 0;
	int na, N;

	N = table.rows();

	Mxf Id;
	Id = Mxf::Identity(N,N);                                       // create a NxN identity matrix of type Mxf (matrix nxn of floats)

	if(set == true){na = (table>0).count();}
	else{na = (table==0).count();}

	Mxf B(N,na);

	// construct matrix of active DOF's
	if(set == true){
	for (int i = 0; i < N; ++i)
	{
		if (table(i) == true) 
		{
			B.block(0,k,N,1) = Id.block(0,i,N,1);
			k++;
		}
	}
	}

	// construct matrix of constraint DOF's
	if(set == false){
	for (int i = 0; i < N; ++i)
	{
		if (table(i) == false) 
		{
			B.block(0,k,N,1) = Id.block(0,i,N,1);
			k++;
		}
	}	
	}

	return B;

}
//---------------------------------------------------
//--------------------------- compute Hessian matrix
//---------------------------------------------------
void Model::hessianInverse(float dt, Vxf R, Vxf &dr){

	int n = NState;
	Mxf S(2*n,2*n);
	Mxf Mi(n,n);

	Mi.noalias() = Mee.householderQr().solve(Mxf::Identity(n,n));

	S.block(0,0,n,n).noalias() = -0.5*dt*Mxf::Identity(n,n);
	S.block(0,n,n,n).noalias() = Mxf::Zero(n,n);
	S.block(n,n,n,n).noalias() = 0.5*dt*Mi*Dee;
	S.block(n,0,n,n).noalias() = 0.5*dt*Mi*Kee;
	S.noalias() += Mxf::Identity(2*n,2*n);

	dr.noalias() = S.partialPivLu().solve(R);
}

//---------------------------------------------------
//----------------------------- build inertia tensor
//---------------------------------------------------
void Model::buildInertiaTensor(){

	Mtt = M6f::Zero(6,6);

	float A  = PI*pow(RADIUS,2);
	float J1 = 0.5*PI*pow(RADIUS,4);
	float J2 = 0.25*PI*pow(RADIUS,4);
	float J3 = 0.25*PI*pow(RADIUS,4);
	
	V6f v;
	v << J1,J2,J3,A,A,A;
	Mtt.diagonal() = ((float)RHO)*v;
}

//---------------------------------------------------
//--------------------------- build stiffness tensor
//---------------------------------------------------
void Model::buildStiffnessTensor(){

	Ktt = M6f::Zero(6,6);

	float A  = PI*pow(RADIUS,2);
	float J1 = 0.5*PI*pow(RADIUS,4);
	float J2 = 0.25*PI*pow(RADIUS,4);
	float J3 = 0.25*PI*pow(RADIUS,4);
	
	float E0 = ((float) EMOD);
	float G0 = ((float) EMOD)/(2*(1+((float) NU)));

	V6f v;
	v << G0*J1,E0*J2,E0*J3,E0*A,G0*A,G0*A;
	Ktt.diagonal() = v;
}

//---------------------------------------------------
//----------------------------- build damping tensor
//---------------------------------------------------
void Model::buildDampingTensor(){
	Dtt = (1.0*MU)*Ktt;
}

//---------------------------------------------------
//----------------------------- build global system
//---------------------------------------------------
void Model::buildGlobalSystem(){

	int n = NState;
	double h, s, ds;

	Mxf K1K(n,n), K2K(n,n);
	Mxf K1M(n,n), K2M(n,n);
	Mxf K1D(n,n), K2D(n,n);

	// initial  matrix
	Kee  = Mxf::Zero(NState,NState);
	Mee  = Mxf::Zero(NState,NState);
	Dee  = Mxf::Zero(NState,NState);
	Cee  = Mxf::Zero(NState,NState);
	Mtee = Mxf::Zero(NState,NState);
	Gee  = Vxf::Zero(NState);

	s = 0.0;
	ds = (1.0*(SDOMAIN))/(1.0*(INTSTEP));

	// do spatial integration
	for (int i = 0; i < INTSTEP; i++){
		systemMatODE(s,K1K,K1M,K1D);
 		systemMatODE(s+(2.0/3.0)*ds,K2K,K2M,K2D);

  		s += ds;
  		Kee.noalias() += (ds/4.0)*(K1K+3.0*K2K);
  		Mee.noalias() += (ds/4.0)*(K1M+3.0*K2M);
  		Dee.noalias() += (ds/4.0)*(K1D+3.0*K2D);
	}

	Mee = ((Mee.array().abs() > 1e-6*Mee.norm()).select(Mee.array(),0.0));
	Kee = ((Kee.array().abs() > 1e-6*Kee.norm()).select(Kee.array(),0.0));
	Dee = ((Dee.array().abs() > 1e-6*Dee.norm()).select(Dee.array(),0.0));

	Mee = Mee.cast<float> ();
	Kee = Kee.cast<float> ();
	Dee = Dee.cast<float> ();
}

//---------------------------------------------------
//---------------------------- build system matrices
//---------------------------------------------------
void Model::systemMatODE(float s, 
	Mxf &K, Mxf &M, Mxf &D){

	Mxf PMat(NDof,NState);

	// evaluate strain-field
	Phi.eval(s,PMat);

	K.noalias() = ((Ba*PMat).transpose())*Ktt*(Ba*PMat);
	M.noalias() = ((Ba*PMat).transpose())*Mtt*(Ba*PMat);
	D.noalias() = ((Ba*PMat).transpose())*Dtt*(Ba*PMat);
}

//---------------------------------------------------
//------------- pressure mapping for 3dof soft robot
//---------------------------------------------------
Mxf Model::pressureMapping(){

	Mxf K(6,3);

	K <<  0,            0,           0,
	      0, -0.5*sqrt(3), 0.5*sqrt(3),
	   -1.0,          0.5,         0.5,
	   -0.0,         -0.0,        -0.0,
	      0,            0,           0,
	      0,            0,           0;


	return ((float)PRS_AREA)*K;
}

