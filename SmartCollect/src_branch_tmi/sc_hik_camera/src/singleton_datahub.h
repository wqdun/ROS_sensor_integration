#ifndef __SINGLETON_DATAHUB_H
#define __SINGLETON_DATAHUB_H

#include <glog/logging.h>

class SingletonDataHub {
private:
    SingletonDataHub();
    ~SingletonDataHub();

    SingletonDataHub(const SingletonDataHub &);
    SingletonDataHub &operator=(const SingletonDataHub &);

    static SingletonDataHub *pSingletonDataHub_;
    double unixTimeMinusGpsTime_;


public:
    static SingletonDataHub &GetInstance();

    void SetUnixTimeMinusGpsTime(double unixTimeMinusGpsTime);
    double GetUnixTimeMinusGpsTime();
};

#endif
