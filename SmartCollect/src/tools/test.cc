#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <pwd.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

// g++ -std=c++11 test.cc -lboost_system -lboost_thread
#define LOG(INFO) cout
// #define LOG(ERROR) cerr

#define BOOST_DATE_TIME_SOURCE
#define BOOST_THREAD_NO_LIB

using namespace std;
using namespace boost;


class HelloWorld
     {
     public:
     HelloWorld()
     {
         std::cout <<
         "Hello world"
         << std::endl;
     }

     ~HelloWorld() {

       std::cout <<
         "Kill Hello" << endl;


     }

 };

boost::mutex io_mutex;
struct ccccc
{
        ccccc(int id) : id(id) { }

        void operator()()
        {
                for (int i = 0; i < 10; ++i)
                {
                        boost::mutex::scoped_lock lock(io_mutex);
                                            HelloWorld hel;

                        usleep(10000);
                        std::cout << id << ": "
                        << i << std::endl;
                }
        }

        int id;
};
int main(int argc, char* argv[])
{
        boost::thread thrd1(ccccc(1));
        boost::thread thrd2(ccccc(2));
        thrd1.join();
        thrd2.join();
        return 0;
}

