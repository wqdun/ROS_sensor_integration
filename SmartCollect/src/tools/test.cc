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

using namespace std;

//获取地址
//返回IP地址字符串
//返回：0=成功，-1=失败
int getLocalIp(std::vector<std::string> &IPs) {
    struct ifconf ifConf;
    ifConf.ifc_len = 512;
    char buf[512];
    ifConf.ifc_buf = buf;

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0) {
        cout << "Socket error: " << sockfd;
        return -1;
    }
    // get all interfaces
    ioctl(sockfd, SIOCGIFCONF, &ifConf);
    close(sockfd);

    struct ifreq *ifReq = (struct ifreq*)buf;
    // get IPs one by one
    int ipCnt = ifConf.ifc_len / sizeof(*ifReq);
    cout << "Got " << ipCnt << " IPs.";
    if(ipCnt > 5) {
        cout << "Got too many IPs: " << ipCnt;
        return -1;
    }

    char *ip;
    for(int i = 0; i < ipCnt; ++i) {
        ip = inet_ntoa(((struct sockaddr_in*)&(ifReq->ifr_addr))->sin_addr);
        if(NULL == ip) {
            cout << "inet_ntoa error: " << ip;
            return -1;
        }
        IPs.push_back(ip);
        ifReq++;
    }

    return 0;
}


int main(void)
{
    char mac[17];

    string masterIp("172.21.11.111");
    std::vector<std::string> IPs;
        if (getLocalIp( IPs ) == 0 )
    {

        cout << IPs[0] << endl;
    }
    else
    {
        printf("无法获取本机IP地址");
    }

    cout << masterIp.substr(0, 6);

    for(auto &IP: IPs) {
        if(0 == IP.find(masterIp.substr(0, 6) ) ) {
            cout << "Got local IP: " << IP;
            break;
        }
    }
    std::string rosIP("");
    cout << "\n" << rosIP.empty() << "\n";


    return 0;
}