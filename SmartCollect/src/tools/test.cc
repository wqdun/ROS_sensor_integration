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


#define LOG(INFO) cout
// #define LOG(ERROR) cerr

#define BOOST_DATE_TIME_SOURCE
#define BOOST_THREAD_NO_LIB

using namespace std;
using namespace boost;

class HelloWorld
{
public:
 void hello()
 {
    std::cout <<
    "Hello world, I''m a thread!"
    << std::endl;
 }
 void start()
 {
  boost::function0< void> f =  boost::bind(&HelloWorld::hello,this);
  boost::thread thrd( f );
  thrd.join();
 }

};
int main(int argc, char* argv[])
{
 HelloWorld hello;
 hello.start();


try
    {
        int i = boost::lexical_cast<int>("");
    }
    catch (boost::bad_lexical_cast& e)
    {
        cout << e.what() << endl;
    }
    int a = lexical_cast<int>("123");
    double b = lexical_cast<double>("");
    std::cout<<a<<std::endl;
    std::cout<<b<<std::endl;

    return 0;
}



