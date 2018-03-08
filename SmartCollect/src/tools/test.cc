#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
// #include <direct.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <errno.h>
#include <pwd.h>

using namespace std;

#define LOG(INFO) cout
// #define LOG(ERROR) cerr

#include <iostream>
#include <boost/thread.hpp>

using namespace std;
using namespace boost;

#define BOOST_DATE_TIME_SOURCE
#define BOOST_THREAD_NO_LIB


//----------------------------------
//函数声明
// g++ test.cc -lboost_system -lboost_thread
//start from the very beginning,and to create greatness
//@author: Chuangwei Lin
//@E-mail：979951191@qq.com
//@brief： boost多线程编译的示例
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

boost::mutex io_mutex;

struct cccccc
{
        cccccc(int id) : id(id) { }

        void operator()()
        {
            boost::mutex::scoped_lock lock(io_mutex);
                for (int i = 0; i < 10; ++i)
                {
                        // boost::mutex::scoped_lock lock(io_mutex);
                        std::cout << id << ": "
                        << i << std::endl;
                }
        }

        int id;
};


void lcw1()
{

    std::cout << "lcw1 is working!" << std::endl;
}

void lcw2()
{

    std::cout << "lcw2 is working!" << std::endl;
}

int main (int argc, char ** argv)
{
    using namespace std;

    using namespace boost;
    using boost::lexical_cast;
    thread thread_1 = thread(lcw1);
    thread thread_2 = thread(lcw2);

    boost::thread thrd1(cccccc(1));
    boost::thread thrd2(cccccc(2));

    thrd1.join();
    thrd2.join();

    thread_2.join();
    thread_1.join();

    int a = lexical_cast<int>("123");
    double b = lexical_cast<double>("");
    std::cout<<a<<std::endl;
    std::cout<<b<<std::endl;
    return 0;
}