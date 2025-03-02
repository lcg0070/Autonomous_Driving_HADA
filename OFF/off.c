//
// Created by User on 2024-09-04.
//


#include <stdio.h>
#include "OFF_processing.hpp"
#include <windows.h>
#include "clustering.h"


double GetWindowTime(void){
    LARGE_INTEGER liEndCounter, liFrequency;
    QueryPerformanceCounter(&liEndCounter);
    QueryPerformanceFrequency(&liFrequency);

    return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);

}
int main()
{
    init_OFF();

    while (1)
    {
        double start_time = GetWindowTime();
        if(OFF_processing())
        {
            break;
        }
        printf("Iter time = %f \n", GetWindowTime()-start_time);

    }
    clear_OFF();

    return 0;
}
