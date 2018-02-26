#ifndef _Lock_H
#define _Lock_H

#include <pthread.h>

//锁接口类
class ILock
{
public:
	virtual ~ILock() {}

	virtual void Lock() const = 0;
	virtual void Unlock() const = 0;
};

//互斥锁类
class CMutex : public ILock
{
public:
	CMutex();
	~CMutex();

	virtual void Lock() const;
	virtual void Unlock() const;
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


