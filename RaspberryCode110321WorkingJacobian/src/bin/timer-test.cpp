

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fstream>
#include <cstring>
#include <string>


using namespace std;

#include "Timer.h"

Timer timer;

void TM_callback()
{
 printf("run--> timer-event \n");
}

void Setup()
{
 printf("Setup...\n\n");
 timer.every(1000,TM_callback,true);
}

int main(int argc, const char *argv[])
{
    Setup();
      	
	while (1)
    {		
	  
	  timer.update();  
	}
return 0;
}


