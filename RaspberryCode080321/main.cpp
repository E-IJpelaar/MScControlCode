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
	double angle;
	double x_pos,y_pos;
	float t;
	
	p0 = mot.PressureCallibration();
	
	double p_set = 1700;
	
	
	mot.tic();		
	
	
	while(mot.toc() < 10){
			
			
			int N = 0;
			
			
			p = mot.readPressure();
			u = mot.PIDcontroller(p_set,p,4096);
			mot.setMotor(1,u);
			t = mot.toc();
			
			

						
			d_p = p - p0;
			double error = p_set - p;
			
			cout << d_p << " " << u << endl;
								    		
			file << t << ";"<<  p0 << ";" << d_p <<  ";" << p << ";" << u   <<  error << ";"<< p_set << "\n";				    
		     		    
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


/*
 * 			if (t < 1){
				u = 0;
				mot.setMotor(1,0);
			}
			else if (t < 26){
				u = 2*341.3333;	
				mot.setMotor(1,u);
			}
			else if  (t > 26){
				u = 0;
				mot.setMotor(1,0);
			}
*/

		
