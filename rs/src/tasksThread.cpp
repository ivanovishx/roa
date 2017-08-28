// =============================================================================
//
//  taskThread.cpp
//
//  Run in Windows VC++
//  Created by Ivan Lozano
//  Research purposes
//
// =============================================================================

#include "tasksThread.h"
#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>
#include <iostream>
// #include "utils.h"
#include <windows.h>
// ............................................................... process_input

using namespace std;


long ScheduleTask::now_ms() {
    // static struct timeb start;
    ftime(&start);
    long timestamp = clock();
    // long timestamp = (long)start.millitm;
    // std::cout << "time: " << timestamp << std::endl;
    // printf("time:%u\n", timestamp);
    return timestamp;
}

void ScheduleTask::thread_work_cicle()
{
    while (true)
    {
        // cout<<"Running inside ScheduleTask::thread_entry() "<<now_ms()<<endl;

        // if ((now_ms() - timer100Hz) > 10) {
        //     timer100Hz = now_ms();
        //     run100hzTasks();
        // }
        // if ((now_ms() - timer50Hz) > 20)
        // {
        //     timer50Hz = now_ms();
        //     run50hzTasks();
        // }
        // if ((now_ms() - timer20Hz) > 50)
        // {
        //     timer20Hz = now_ms();
        //     run20hzTasks();
        // }
        if ((now_ms() - timer10Hz) > 100)
        {
            timer10Hz = now_ms();
            run10hzTasks();
        }
        if ((now_ms() - timer5Hz) > 200)
        {
            timer5Hz = now_ms();
            run5hzTasks();
        }
        if ((now_ms() - timer2Hz) > 500)
        {
            timer2Hz = now_ms();
            run2hzTasks();
        }
        if ((now_ms() - timer1Hz) > 1000)
        {
            timer1Hz = now_ms();
            run1hzTasks();
        }
        if ((now_ms() - timer5Seg) > 5000)
        {
            timer5Seg = now_ms();
            run5SegTasks();
        }

        Sleep(80);
    }
}



bool ScheduleTask::run100hzTasks()
{
    return 1;
}
bool ScheduleTask::run50hzTasks()
{
    cout << "Running inside the run50hzTasks" << endl;
    return 1;
}
bool ScheduleTask::run20hzTasks()
{
    return 1;
}
bool ScheduleTask::run10hzTasks()
{
    cout << "Running inside the run10hzTasks" << endl;
    return 1;
}
bool ScheduleTask::run5hzTasks()
{
    cout << "Running inside the run5hzTasks" << endl;
    return 1;
}
bool ScheduleTask::run2hzTasks()
{
    std::cout << "Running inside the run2hzTasks" << std::endl;
    // printSpeed(api);
    return 1;
}
bool ScheduleTask::run1hzTasks()
{
    std::cout << "Running inside the run1hzTasks" << std::endl;
    // system("clear");
    return 1;
}
bool ScheduleTask::run5SegTasks()
{
    cout << "Running inside the 5 Secs Task" << endl;
    // system("clear");
    return 1;
}

// void ScheduleTask::printSpeed(CoreAPI* api)
// {
//       //TODO: put a contant timer every 500ms to print speed: x,y,z,yaw,TRUE_vector, and acc: x,y,z,yaw,TRUE_vector

//             std::cout<<"-------------printSpeed----------------------------------"<<std::endl;


// }
