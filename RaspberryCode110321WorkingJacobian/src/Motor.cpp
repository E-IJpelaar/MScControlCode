#include "Motor.h"



using namespace std;
using namespace Eigen;

ofstream logger;
Timer hltimer;

//---------------------------------------------------
//-------------------------------- class constructor
//---------------------------------------------------
Motor::Motor(const char* str){
	
  	wiringPiSetup();
	ConfigFile cf(str);

	MOTOR_ENABLE = static_cast<bool>(cf.Value("options","MOTOR_ENABLE"));
	WRITE_OUTPUT = static_cast<bool>(cf.Value("options","WRITE_OUTPUT"));
	PWM_FREQ 	 = static_cast<int>(cf.Value("options","PWM_FREQ"));   
	PWM_MAX      = static_cast<int>(cf.Value("options","PWM_MAX"));

	PIN_BASE     = static_cast<int>(cf.Value("adress","PIN_BASE"));
	PCA9685_ID   = static_cast<char>(cf.Value("adress","PCA9685_ID"));
	MPU6050_ID   = static_cast<char>(cf.Value("adress","MPU6050_ID"));
	MPRLS25_ID   = static_cast<char>(cf.Value("adress","MPRLS25_ID"));
	
	float Ka = static_cast<float>(cf.Value("imufilter","KA"));
	float Kg = static_cast<float>(cf.Value("imufilter","KG"));
	
	UPDATE_RATE_HIGHLEVEL = static_cast<int>(cf.Value("options","HIGHLEVEL"));
	
	M1.EN = static_cast<int>(cf.Value("pinout","EN_A")) + PIN_BASE;
	M2.EN = static_cast<int>(cf.Value("pinout","EN_B")) + PIN_BASE;
	M3.EN = static_cast<int>(cf.Value("pinout","EN_C")) + PIN_BASE;
	M4.EN = static_cast<int>(cf.Value("pinout","EN_D")) + PIN_BASE;
	M1.INA = static_cast<int>(cf.Value("pinout","IN1")) + PIN_BASE;
	M1.INB = static_cast<int>(cf.Value("pinout","IN2")) + PIN_BASE;
	M2.INA = static_cast<int>(cf.Value("pinout","IN3")) + PIN_BASE;
	M2.INB = static_cast<int>(cf.Value("pinout","IN4")) + PIN_BASE;
	M3.INA = static_cast<int>(cf.Value("pinout","IN5")) + PIN_BASE;
	M3.INB = static_cast<int>(cf.Value("pinout","IN6")) + PIN_BASE;
	M4.INA = static_cast<int>(cf.Value("pinout","IN7")) + PIN_BASE;
	M4.INB = static_cast<int>(cf.Value("pinout","IN8")) + PIN_BASE;
	
	M1.KP = static_cast<float>(cf.Value("control","KP1"));
	M2.KP = static_cast<float>(cf.Value("control","KP2"));
	M3.KP = static_cast<float>(cf.Value("control","KP3"));
	M4.KP = static_cast<float>(cf.Value("control","KP4"));
	
	
	Kp = static_cast<double>(cf.Value("pressurePI","Kp"));
	Ki = static_cast<double>(cf.Value("pressurePI","Ki"));
	Kd = static_cast<double>(cf.Value("pressurePI","Kd"));

	

	cout << setprecision(PRECISION) << endl;
  	
  	cout << "Connecting to arduino board..." << endl;
  	
  	  	
  	if ((fd = serialOpenB ("/dev/ttyUSB0", 115200,10)) < 0)
	{
	fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	//cout << "Can not connect with Arduino" << endl;
    //return 1 ;
	}
	
	if (fd_pca = pca9685Setup(PIN_BASE,PCA9685_ID,50)) {
		cout << "Unable to access pca9685" << endl;
	}
	
	pca9685PWMReset(fd_pca);

	
	ofstream file;
	file.open("soro.log", ofstream::out | ofstream::trunc);
	file.close();
	
	logger.open("soro.log", ios_base::app | ios::binary);
	
	// set clock rate
	dt = UPDATE_RATE_HIGHLEVEL*1e-6;
	 
}
	
//---------------------------------------------------
//---------------------------------------- set motor
//---------------------------------------------------

void Motor::setMotor(int id, float pwm){
	
	if (pwm > (float)PWM_MAX){
		pwm = (float)PWM_MAX;
	}
	if (pwm < 0){pwm = 0;}
	
	if (id == 1){		
		if(pwm > 0){pwm = pwm;}
		pwmWrite(M1.INA, 4095);
		pwmWrite(M1.INB, 0);
		pwmWrite(M1.EN, pwm);
	}
	if (id == 2){		
		if(pwm > 0){pwm = pwm;}
		pwmWrite(M2.INA, 4095);
		pwmWrite(M2.INB, 0);
		pwmWrite(M2.EN, pwm);
	}
	if (id == 3){		
		if(pwm > 0){pwm = pwm;}
		pwmWrite(M3.INA, 4095);
		pwmWrite(M3.INB, 0);
		pwmWrite(M3.EN, pwm);
	}
	if (id == 4){		
		if(pwm > 0){pwm = pwm;}
		pwmWrite(M4.INA, 4095);
		pwmWrite(M4.INB, 0);
		pwmWrite(M4.EN, pwm);
	}

}

double Motor::readPressure(){
	

	double new_pressure;
	double pressure;
	
	serialPutchar(fd,'1');
	
	int buf_length = serialDataAvail(fd);
	//cout << buf_length << endl;
	char buf[buf_length];
	for(int i = 0; i<buf_length; i++){
	buf[i] = serialGetchar(fd);
	}
	new_pressure = strtod(buf, &endptr);
	serialFlush(fd);
	
	if(new_pressure == 0){
		pressure = old_pressure;
		}
	else{
		pressure = new_pressure;
		old_pressure = new_pressure;
		}
	return pressure;
	  

	
}

//---------------------------------------------------
//----------------------------------------- read MPU
//---------------------------------------------------
double Motor::readMPU(){
	
	double new_a;
	double a;
	
	serialPutchar(fd,'2');		
	//int buf_length = serialDataAvail(fd);
	//cout << buf_length << endl;
	//char buf[buf_length];
	char buf[BUFSIZE];
	for(int i = 0; i<BUFSIZE; i++){
	buf[i] = serialGetchar(fd);
	}
	new_a = strtod(buf, &endptr);
	serialFlush(fd);
	
	if(new_a == 0){
		a = old_a;
		}
	else{
		a = new_a;
		old_a = new_a;
		}
	return a;
	
}


//---------------------------------------------------
//------------------------ read x position Pixy
//---------------------------------------------------
double Motor::getX(){
	
	double new_X;
	double X;
	
	serialPutchar(fd,'3');	
	
	//int buf_length  = 10;	
	int buf_length = serialDataAvail(fd);
	//cout << buf_length << endl;
	char buf[buf_length];
	for(int i = 0; i<buf_length; i++){
	buf[i] = serialGetchar(fd);
	}
	new_X = strtod(buf, &endptr);
	serialFlush(fd);
	
	if(new_X == 0){
		X = old_X;
		}
	else{
		X = new_X;
		old_X = new_X;
		}
	return X;

	
}

//---------------------------------------------------
//------------------------ read y position Pixy
//---------------------------------------------------
double Motor::getY(){
	
	double new_Y;
	double Y;
	
	serialPutchar(fd,'4');	
	
	
	//int buf_length  = 10;	
	int buf_length = serialDataAvail(fd);
	//cout << buf_length << endl;
	char buf[buf_length];
	for(int i = 0; i<buf_length; i++){
	buf[i] = serialGetchar(fd);
	}
	new_Y = strtod(buf, &endptr);
	serialFlush(fd);
	
	if(new_Y == 0){
		Y = old_Y;
		}
	else{
		Y = new_Y;
		old_Y = new_Y;
		}
	return Y;

	
}

//---------------------------------------------------
//----------------------------- Pressure Callibration
//---------------------------------------------------

double Motor::PressureCallibration(){
	
	double p_cur;
	p_cur = readPressure();
	while(p_cur == 0){
		p_cur = readPressure();
		}
	return p_cur;
	
}

//---------------------------------------------------
//----------------------------- Pressure Controller
//---------------------------------------------------
double Motor::PIDcontroller(double setpoint,double current_value,double I_max){
	
	
	double _max = 4096;
	double _min = 0;
	
	
	
	// Calculate error
    double error = setpoint - current_value;
	cout << "e: " << error << endl;
    // Proportional term
    double Pout = Kp * error;
	cout << "P: " << Pout << endl;
    // Integral term
    integral += error * dt;
    double Iout = Ki * integral;
    
    
    // Integrator clamping
    if (Iout >= I_max){
		Iout = 0;
		}
	else {
		Iout = Iout;
		}
    cout << "I: " << Iout << endl;

    // Derivative term
    double derivative = (error - pre_error) / dt;
    double Dout = Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;
	cout << "u: " << output<< endl;
    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    pre_error = error;

    return output;	
	
	
}


//---------------------------------------------------
//---------------------------- build Jacobian matrix
//---------------------------------------------------

void Motor::buildJacobian(float se, Mxf &J, Mxf &Jt) {

		int n = 2;
		double ds,s;
		
		
		Mxf K1J(6,n), K2J(6,n);
		Mxf K1Jt(6,n), K2Jt(6,n);
		M6f adeta,AdgInv;
		V13f K1,K2;
		V13f x,dx;
		
		// initial Jacobian matrix
		
		J.setZero();
		Jt.setZero();
		x.setZero();
		x(0) = 1.0;
		
		
		// compute forward integration step
		ds  = (1.0*(se))/(1.0*(SPACESTEP));
		s = 0.0;
				
		// spatial integration
		for(int i = 0; i < SPACESTEP; i++){
			
			jacobiODE(s,x,K1,K1J,K1Jt);
			jacobiODE(s+(2.0/3.0)*ds,x+(2.0/3.0)*ds*K1,K2,K2J,K2Jt);
			
						
			s += ds;
			x.noalias() += (ds/4.0)*(K1 +3.0*K2);
			J.noalias() += (ds/4.0)*(K1J + 3.0*K2J);
			Jt.noalias() += (ds/4.0)*(K1Jt + 3.0*K2Jt);
		}
			
		// 	return configuration
		g.noalias() = x.block(0,0,7,1);
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

void Motor::jacobiODE(float s, V13f x, V13f &dx, Mxf &dJ, Mxf &dJt){
	
	Mxf PMat(NDof,NState);
	M6f adxi,Adg,adeta;
	
    Phi.set(NMode,NDof,"legendre");
	
	Vxf q(2,1);
	Vxf dq(2,1);
	
    q  << 0.0,0.1;
	dq << 0.0,0.0;
	
	Mxf Ba(6,2);
	Vxf Xi(6,1);
	
	Ba << 0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0;
	Xi0 << 0.0,0.0,0.0,1.0,0.0,0.0;
		
	
	// evaluate strain field
	Phi.eval(s,PMat);
	xi.noalias() = (Ba*PMat)*q + Xi0;
	dxi.noalias() = (Ba*PMat)*dq;
	
	
	// decompostion configuration space
	quat.noalias()= x.block(0,0,4,1);
	eta.noalias() = x.block(7,0,6,1);
	xiK.noalias() = xi.block(0,0,3,1);
	xiE.noalias() = xi.block(3,0,3,1);
	
		
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
//---------------------------------- reset all motors
//---------------------------------------------------

void Motor::reset(){
	for(int i = 0; i < 4; i++){
		setMotor(i+1,0);	
	}
}	

//---------------------------------------------------
//------------------------------- tic - toc functions
//---------------------------------------------------
void Motor::tic(){
	t = 0.0;
	hltimer = hltimer - hltimer;
}

float Motor::toc(){
	return t;
}

void Motor::updateClock(){
	t += dt;
	hltimer = hltimer - UPDATE_RATE_HIGHLEVEL;
}



int Motor::serialOpenB (const char *device, const int baud, const int max)
{
    struct termios options ;
    int FD = serialOpen(device, baud);
    // Get and modify current options:
    tcgetattr (FD, &options) ;

    options.c_lflag |= ICANON;  // set canonical mode (line by line)
    options.c_iflag |= IGNCR;   // ignore CR on input
    options.c_cc [VMIN] = max-1;    // return if max-1 bytes received
    options.c_cc [VTIME] = 0;   // no timeout

    tcsetattr (FD, TCSANOW | TCSAFLUSH, &options) ;

    usleep (10000) ;    // 10mS

    return FD ;
}
