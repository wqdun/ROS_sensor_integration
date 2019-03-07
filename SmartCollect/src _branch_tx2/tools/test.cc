#include <iostream>
#include <ctime>
#include <string>
#include <sys/time.h>
#include <time.h>

using namespace std;

// std::string unixTime2String(uint64_t unixTime) {
//     tm tmNow = { 0 };
//     localtime_r((time_t *)&unixTime, &tmNow);


//     cout << tmNow.tm_hour * 100 + tmNow.tm_min;
//     // strftime(strTime, bufLen - 1, "%Y-%m-%d %H:%M:%S", &tmNow);
//     // strTime[bufLen - 1] = '\0';
// }

int main(void)
{
    struct timeval tv;
    struct timezone tz;
    gettimeofday (&tv, &tz);

    tv.tv_sec = 1546910000;
    tv.tv_usec = 0;

    settimeofday(&tv, NULL);

    return 0;
}


// time_t tt = time(NULL);
//     tm *t= localtime(&tt);
//     char nowTime[50];
//     (void)sprintf(nowTime, "%02d%02d%02d", t->tm_hour, t->tm_min, t->tm_sec);
//     fileName += nowTime;

//     time_t now = time(NULL);
//   tm tmNow = { 0 };
//   localtime_r(&now, &tmNow);
//   const int nowMinute = tmNow.tm_hour * 100 + tmNow.tm_min;
//   if(nowMinute != lastMinute_) {
//     lastMinute_ = nowMinute;
//     isGonnaCreateNewLidarFile = true;
//   }