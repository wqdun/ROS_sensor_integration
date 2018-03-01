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


int main()
{
    LOG(INFO) << __FUNCTION__ << " start.";

    LOG(INFO) << "getenv(DDDDDDDDDD): " << getenv("DDDDDDDDDD");

    int err(0);
    const std::string rvizExe("rviz &");
    FILE *fpin;
    if(NULL == (fpin = popen("rviz &", "r") ) ) {
        LOG(ERROR) << "Failed to " << rvizExe;
        exit(1);
    }
    if(0 != (err = pclose(fpin) ) ) {
        LOG(ERROR) << "Failed to " << rvizExe << ", returns: " << err;
        exit(1);
    }
    LOG(INFO) << "Run: " << rvizExe << " end.";


    // // put ENV again, or ROS_MASTER_URI is NULL
    // const std::string masterIp(pMaterIpEdit_->text().toStdString() );
    // std::string rosMaterUri("ROS_MASTER_URI=http://" + masterIp + ":11311");
    // char *rosMaterUriData = string_as_array(&rosMaterUri);
    // int err = putenv(rosMaterUriData);
    // LOG(INFO) << "Put env: "<< rosMaterUriData << " returns: " << err;

    // const std::string rvizExe("/opt/ros/indigo/bin/rviz &");
    // LOG(INFO) <<"Run " << rvizExe;

    // FILE *fpin;
    // if(NULL == (fpin = popen(rvizExe.c_str(), "r") ) ) {
    //     LOG(ERROR) << "Failed to " << rvizExe;
    //     exit(1);
    // }
    // if(0 != (err = pclose(fpin) ) ) {
    //     LOG(ERROR) << "Failed to " << rvizExe << ", returns: " << err;
    //     exit(1);
    // }
    // LOG(INFO) << "Run: " << rvizExe << " end.";
    return 1;
}