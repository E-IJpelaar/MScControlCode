#ifndef TICTOC_H
#define TICTOC_H

#include <iostream>
#include <chrono>
#include <cmath>

#define TIMEFRAME 5000

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;
typedef std::chrono::microseconds microseconds;
typedef std::chrono::nanoseconds nanoseconds;
static Clock::time_point t0 = Clock::now();
static Clock::time_point lpt = Clock::now();

bool loopactive = false;

using namespace std::chrono;
using namespace std;

void tic()
{
 t0 = Clock::now();
}

double toc()
{
    Clock::time_point t1 = Clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t1 - t0);

    return time_span.count();
}

bool looper()
{
    bool flag = true;
    
    if(!loopactive){ 
        loopactive = true;
        lpt = Clock::now();
    }
    
    Clock::time_point t1 = Clock::now();
    microseconds time_span = duration_cast<microseconds>(t1 - lpt);
        
    if (time_span.count() >= TIMEFRAME){flag = false;}
    if(!flag){loopactive = false;}
    
    return flag;
}

#endif
