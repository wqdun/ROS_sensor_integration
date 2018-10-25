#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <signal.h>

#include "../../../sc_lib_public_tools/src/thread_pool.h"

class task
{
private:
    int number;

public:
    task(int num):// , const bool &_isRunning):
        number(num)
    {
        std::cout << __FUNCTION__ << " start.\n";
    }
    ~task() {
        std::cout << __FUNCTION__ << "\n";
    }

    void doit() {//const bool &_isRunning) {
        // while(_isRunning) {
        //     std::cout << __FUNCTION__ << " " << _isRunning << "\n";
        //     sleep(1);
        // }
        std::cout << __FUNCTION__ << "\n";
        usleep(2000);
        std::cout << "this is the " << number << "th task.\n";
    }
};


bool isRunning = true;

void CtrlCHandler(int signo) {
    std::cout << "Control C Hit.\n";
    isRunning = false;
}

int main() {
    // signal(SIGINT, CtrlCHandler);
    // task tasker(0);// , isRunning);
    // tasker.doit();// isRunning);

    std::cout << "this is task.\n";
    // return 0;

    threadpool<task> pool(2, 4);
    pool.start();

    task *ta;
    for(int i = 0; i < 8; ++i) {
        ta = new task(i);
        pool.append_task(ta);
        // usleep(100000);
    }

    // for(auto &pool_thread: pool.all_threads) {
    //     void *tret;
    //     int err = pthread_join(pool_thread, &tret);
    //     if(0 != err) {
    //         std::cout << "Error join.\n";
    //         exit(-1);
    //     }
    //     std::cout << "Return " << (long)tret << "\n";
    // }


    usleep(10000);
    std::cout << "Close the thread pool.\n";
    pool.stop();
    return 0;
}
