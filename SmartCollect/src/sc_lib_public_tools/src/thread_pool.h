#ifndef _PTHREAD_POOL_
#define _PTHREAD_POOL_

#include <list>
#include <stdio.h>
#include <exception>
#include <errno.h>
#include <pthread.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include "locker.h"

template<class T>
class threadpool
{
private:
    int thread_number;
    int max_task_number;

    std::list<T *> task_queue;
    mutex_locker queue_mutex_locker;
    sem_locker queue_sem_locker;
    bool is_stop;
    int busyWorker_;
public:
    threadpool(int thread_num = 20, int max_task_num = 30);
    ~threadpool();
    bool append_task(T *task);
    void start();
    void stop();

    std::vector<pthread_t> all_threads;
private:
    static void *worker(void *arg);
    void run();
};

template <class T>
threadpool<T>::threadpool(int thread_num, int max_task_num):
    thread_number(thread_num),
    max_task_number(max_task_num),
    is_stop(false),
    busyWorker_(0)
{
    all_threads.clear();
    if((thread_num <= 0) || max_task_num <= 0) {
        std::cout << "thread pool can't init because thread_number = 0 or max_task_number = 0.\n";
        exit(-1);
    }

    for(size_t i = 0; i < thread_num; ++i) {
        pthread_t _thread;
        all_threads.push_back(_thread);
    }
    if(all_threads.empty()) {
        std::cout << "Can't initial thread pool because thread array can't new.";
        exit(-1);
    }
}

template <class T>
threadpool<T>::~threadpool()
{
    std::cout << __FUNCTION__ << " start.\n";
    all_threads.clear();
    is_stop = true;
}

template <class T>
void threadpool<T>::stop()
{
    is_stop = true;
    // queue_sem_locker.add();
}

template <class T>
void threadpool<T>::start() {
    std::cout << __FUNCTION__ << " start.\n";
    for(int i = 0; i < thread_number; ++i) {
        std::cout << "create the " << i << "th thread.\n";
        if(pthread_create(&(all_threads[i]), NULL, worker, this) != 0) {
            all_threads.clear();
            throw std::exception();
        }

        // if(pthread_detach(all_threads[i])) {
        //     all_threads.clear();
        //     throw std::exception();
        // }
    }
}

template <class T>
bool threadpool<T>::append_task(T *task) {
    std::cout << __FUNCTION__ << " start.\n";
    ++busyWorker_;
    queue_mutex_locker.mutex_lock();
    std::cout << "task_queue.size(): " << task_queue.size() << "\n";
    if(task_queue.size() >= max_task_number) {
        queue_mutex_locker.mutex_unlock();
        return false;
    }
    task_queue.push_back(task);
    queue_mutex_locker.mutex_unlock();

    queue_sem_locker.add();
    std::cout << "queue_sem_locker.add()\n";

    return true;
}

template <class T>
void *threadpool<T>::worker(void *arg) {
    std::cout << __FUNCTION__ << " start, pthreadId: " << (unsigned long)pthread_self() << ".\n";
    threadpool *pool = (threadpool *)arg;
    pool->run();
    return pool;
}

template <class T>
void threadpool<T>::run() {
    std::cout << __FUNCTION__ << " start.\n";
    while(!is_stop) {
        std::cout << "Waiting 4 semaphore.\n";
        queue_sem_locker.wait();
        std::cout << "Got a semaphore.\n";
        if(errno == EINTR) {
            std::cout << "error.\n";
            continue;
        }

        queue_mutex_locker.mutex_lock();
        if(task_queue.empty()) {
            queue_mutex_locker.mutex_unlock();
            continue;
        }
        T *task = task_queue.front();
        task_queue.pop_front();
        queue_mutex_locker.mutex_unlock();
        if(!task) {
            continue;
        }
        std::cout << "pthreadId = " << (unsigned long)pthread_self() << "\n";
        task->doit();
        delete task;
        task = NULL;
        --busyWorker_;
        std::cout << busyWorker_ << " busyWorker_.\n";
    }

    std::cout << "Close " << (unsigned long)pthread_self() << "\n";
}

#endif
