#include "singleton_datahub.h"

SingletonDataHub::SingletonDataHub() {
    LOG(INFO) << __FUNCTION__ << " start.";

    unixTimeMinusGpsTime_ = 0;
}

SingletonDataHub::~SingletonDataHub() {
    LOG(INFO) << __FUNCTION__ << " start.";
}

SingletonDataHub &SingletonDataHub::GetInstance() {
    static SingletonDataHub singletonDataHub;

    return singletonDataHub;
}

void SingletonDataHub::SetUnixTimeMinusGpsTime(double unixTimeMinusGpsTime) {
    DLOG(INFO) << __FUNCTION__ << " start.";
    unixTimeMinusGpsTime_ = unixTimeMinusGpsTime;
}

double SingletonDataHub::GetUnixTimeMinusGpsTime() {
    DLOG(INFO) << __FUNCTION__ << " start.";
    return unixTimeMinusGpsTime_;
}

