#include "lock.h"


//动态方式初始化互斥锁
CMutex::CMutex()
{
    pthread_mutex_init(&m_mutex, NULL);
    //pthread_mutexattr_settype(&m_mutex, PTHREAD_MUTEX_ERRORCHECK_NP);
}

//注销互斥锁
CMutex::~CMutex()
{
    pthread_mutex_destroy(&m_mutex);
}

//确保拥有互斥锁的线程对被保护资源的独自访问
int CMutex::Lock() const
{
    return pthread_mutex_lock(&m_mutex);
}

//try to lock
int CMutex::Trylock(void)
{
    this->ret = pthread_mutex_trylock(&m_mutex);
    return this->ret;
}

//释放当前线程拥有的锁，以使其它线程可以拥有互斥锁，对被保护资源进行访问
int CMutex::Unlock() const
{
    return pthread_mutex_unlock(&m_mutex);
}

//利用C++特性，进行自动加锁
CMyLock::CMyLock(const ILock& m) : m_lock(m)
{
    m_lock.Lock();
}

//利用C++特性，进行自动解锁
CMyLock::~CMyLock()
{
    m_lock.Unlock();
}

