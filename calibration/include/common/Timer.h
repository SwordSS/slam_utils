#ifndef _TIMER_H_
#define _TIMER_H_

#include <time.h>

class Timer
{
public:
    Timer(double time_increment_min_value_ = 0.01):
    time_increment_min_value(time_increment_min_value_)
    {
        last = clock_t();
    }

    bool IsReachTime()
    {
        clock_t now = clock();
        double time_increment = double(now-last)/CLOCKS_PER_SEC;
        if(time_increment>time_increment_min_value)
        {
            return true;
        }
        else 
        {
            return false;
        }
    }

    double GetThroughTime()
    {
        clock_t now = clock();
        return double(now-last)/CLOCKS_PER_SEC;
    }

    void UapdateLastTime()
    {
         last = clock();
    }

private:
    double time_increment_min_value;
    clock_t last;
};

#endif 