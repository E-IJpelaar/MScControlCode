//#define NDEBUG

#include "src/Motor.cpp"


ofstream file;

int main(int argc, char** argv){
	
	char receive[7]; 
	Motor mot(argv[1]);
	
	mot.setMotor(1, 0);
	delay(2000);
 
	
	// cleans log file
	file.open("PIcontrol.txt", ofstream:: out | ofstream::trunc);
	file.close();
	
	// opens log file
	file.open("PIcontrol.txt", ios_base::app | ios::binary);
	
	
	double p,u,p0,d_p;
	double angle,x,y;
	double x_pos,y_pos;
	float t;
	
	p0 = mot.PressureCallibration();
	
	double p_set = 1700;
	
	
	mot.tic();	
	
		
	Mxf J(6,2);
	Mxf Jt(6,2);
	
	while(mot.toc() < 10){
			
			
			int N = 0;
			
			mot.buildJacobian(0.0645 , J, Jt );
			
			//p = mot.readPressure();
			//angle = mot.readMPU();
			//x = mot.getX();
			//y = mot.getY();
			
			t = mot.toc();
			
			cout << J << endl;

		   // cout << "Pressure: " << p << endl;
		    //cout << "Angle: " << angle << endl;
		    //cout << "X position: " << y << endl; 
		   // cout << "Y position: " << x << endl; 
								    		
			//file << t << ";"<<  p0 << ";" << d_p <<  ";" << p << ";" << u   <<  error << ";"<< p_set << "\n";				    
		     		    
		    while(hltimer <= mot.UPDATE_RATE_HIGHLEVEL){
							N++;	
			}
		    	
		    				    		
		mot.updateClock();
		cout << t << " " << N << endl;
	}
		
	mot.reset();
		
	serialClose(mot.fd);
	   
	return 0;
}


		
