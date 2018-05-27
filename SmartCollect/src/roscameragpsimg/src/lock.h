#ifndef _Lock_H
#define _Lock_H

#include <pthread.h>

//锁接口类
class ILock
{
public:
    virtual ~ILock() {}

    virtual int Lock() const = 0;
    virtual int Unlock() const = 0;
};

//互斥锁类
class CMutex : public ILock
{
public:
    CMutex();
    ~CMutex();

    virtual int Lock() const;
    virtual int Unlock() const;
    int Trylock(void);

private:
    mutable pthread_mutex_t m_mutex;
public:
    int ret;
};

//锁
class CMyLock
{
public:
    CMyLock(const ILock&);
    ~CMyLock();

private:
    const ILock& m_lock;
};


#endif


