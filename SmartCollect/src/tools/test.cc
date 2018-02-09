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
#include <string.h>

   #include <stdio.h>
// #include <direct.h>

using namespace std;

int run_cmd(char *cmd, char *result)
{
    FILE *pp;
    int iRet = 0;
    if( (pp = popen(cmd, "r")) == NULL )
    {
        printf("popen() error!\n");
        exit(1);
    }
    fgets(result, 1000, pp);
        printf("%s", result);
    iRet = pclose(pp);
    return iRet;
}
int xargs_test(char *result)
{
    char *cmd;
    int iRet = asprintf(&cmd, "ls -lh /root/Project/programing/bak/ |grep a.out  | awk -F'->' '{print $1}' | awk -F' ' '{print $9}' | xargs rm ");
    if(iRet == -1)
        return iRet;
    return run_cmd(cmd, result);
}
int main()
{
    const size_t MAX_LINE = 1000;

    char result[MAX_LINE];
    FILE *fpin;
    const std::string cmd("ls *.sh");
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) ) {
        printf("popen() error!\n");
        return 1;
    }
    int i = 0;
    while(1) {
        if(NULL == fgets(result, MAX_LINE, fpin) ) {
            break;
        }
        cout << ++i << endl;
        cout << result;

    }

    if(0 != pclose(fpin) ) {
        printf("Failed.");
        return 1;
    }



    char *ros_master_uri_env;
    if(!(ros_master_uri_env = getenv("ROS_MASTER_URI") ) ) {
        LOG(WARNING) << "Failed to get ROS_MASTER_URI: " << ros_master_uri_env;
    }
    else {
        LOG(INFO) << "ROS_MASTER_URI: " << ros_master_uri_env;
    }


    char buffer[1000];
    //也可以将buffer作为输出参数
    if(NULL == (getcwd(buffer, sizeof(buffer) ) ) ) {
        perror("getcwd error");
    }
    else
    {
        printf("%s\n%ld\n", buffer, sizeof(buffer));
    }




//         time_t now = time(NULL);
//     tm tmNow = { 0 };
//     localtime_r(&now, &tmNow);
//     char nowTime[50];
//     (void)sprintf(nowTime, "%02d%02d%02d", tmNow.tm_hour, tmNow.tm_min, tmNow.tm_sec);
// //      p = gmtime(&timep);
//         // p->tm_mday = p->tm_mday + 20;
//     cout << nowTime << "\n";
//         sprintf(nowTime, "%04d%02d%02d\n", (1900+ tmNow.tm_year), (1 + tmNow.tm_mon),tmNow.tm_mday);
//         std::string date_(nowTime);
//     cout << date_.substr(2) << endl;



    return 0;
}
