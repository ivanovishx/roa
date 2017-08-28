// =============================================================================
//
//  taskThread.h
//
//  Run in Windows VC++
//  Created by Ivan Lozano
//  Research purposes
//
// =============================================================================


#ifndef __SHEDULETASK_HPP__
#define __SHEDULETASK_HPP__

#define HAVE_STRUCT_TIMESPEC
#include <pthread.h>
// #include "thread.h"
#include <stdio.h>
#include <sys\timeb.h>

using namespace std;

class ScheduleTask
{

private:


    long timerHz;
    long timer100Hz;
    long timer50Hz;
    long timer20Hz;
    long timer10Hz;
    long timer5Hz;
    long timer2Hz;
    long timer1Hz;
    long timer5Seg;

    bool run100hzTasks();
    bool run50hzTasks();
    bool run20hzTasks();
    bool run10hzTasks();
    bool run5hzTasks();
    bool run2hzTasks();
    bool run1hzTasks();
    bool run5SegTasks();

    struct timeb start;


// Threat:
    pthread_t _thread;
    static void * thread_call(void * t) {

        ((ScheduleTask *)t)->thread_work_cicle();
        return NULL;
    }

    bool create()
    {
        bool a = 0;
        return (pthread_create(&_thread, NULL, thread_call, this) == 0);
           }

    void wait_second_join()
    {
        (void) pthread_join(_thread, NULL);
    }


protected:
    // virtual bool server_input_process( char *buff, size_t cb );
    // virtual bool server_output_process();

public:


    long now_ms();

    ScheduleTask()
    {


        timer100Hz = now_ms();
        timer50Hz = timer100Hz;
        timer20Hz = timer100Hz;
        timer10Hz = timer100Hz;
        timer5Hz = timer100Hz;
        timer2Hz = timer100Hz;
        timer1Hz = timer100Hz;
        timer5Seg = timer100Hz;
        create(); // Create Thread -> calling thread_work_cicle()

    }

    void thread_work_cicle();

};



#endif // __SHEDULE_TASK_HPP__