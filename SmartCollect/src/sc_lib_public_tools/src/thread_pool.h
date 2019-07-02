#ifndef _PTHREAD_POOL_
#define _PTHREAD_POOL_

#include <glog/logging.h>

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
    is_stop(false)
{
    all_threads.clear();
    if((thread_num <= 0) || max_task_num <= 0) {
        LOG(ERROR) << "thread pool can't initial because thread_number = 0 or max_task_number = 0.";
        exit(-1);
    }

    for(size_t i = 0; i < thread_num; ++i) {
        pthread_t _thread;
        all_threads.push_back(_thread);
    }
    if(all_threads.empty()) {
        LOG(ERROR) << "Can't initial thread pool because thread array can't new.";
        exit(-1);
    }
}

template <class T>
threadpool<T>::~threadpool()
{
    LOG(INFO) << __FUNCTION__ << " start.";
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
    DLOG(INFO) << __FUNCTION__ << " start.";
    for(int i = 0; i < thread_number; ++i) {
        LOG(INFO) << "Create the " << i << "th thread.";
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
    queue_mutex_locker.mutex_lock();
    LOG_EVERY_N(INFO, 100) << "task_queue.size(): " << task_queue.size();
    if(task_queue.size() >= max_task_number) {
        LOG(WARNING) << "Need more worker, task_queue too big: " << task_queue.size();
        queue_mutex_locker.mutex_unlock();
        return false;
    }
    task_queue.push_back(task);
    queue_mutex_locker.mutex_unlock();

    queue_sem_locker.add();
    return true;
}

template <class T>
void *threadpool<T>::worker(void *arg) {
    DLOG(INFO) << __FUNCTION__ << " start, pthreadId: " << (unsigned long)pthread_self();
    threadpool *pool = (threadpool *)arg;
    pool->run();
    return pool;
}

template <class T>
void threadpool<T>::run() {
    LOG(INFO) << __FUNCTION__ << " start.";
    while(!is_stop) {
        queue_sem_locker.wait();
        if(errno == EINTR) {
            LOG(ERROR) << "errno == EINTR";
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
        task->doit();
        delete task;
        task = NULL;
    }

    LOG(INFO) << "Close " << (unsigned long)pthread_self();
}

#endif
