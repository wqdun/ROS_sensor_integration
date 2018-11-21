#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <arpa/inet.h>

#define MAXLINE 1024
const size_t POSITION_PACKET_SIZE = 2000;


// static bool parsePositionPkt(const char *buff, velodyne_msgs::Velodyne2Center &parsedRes) {
//     ROS_DEBUG_STREAM(__FUNCTION__ << " start.");
//     if('$' != buff[206]) {
//       ROS_INFO_STREAM("No position packet received: " << std::hex << (int)buff[206]);
//       // PPS_STATUS[0] is "No PPS", refer infor_process.h
//       parsedRes.pps_status_index = 0;
//       // A validity - A-ok, V-invalid, refer VLP-16 manual
//       parsedRes.is_gprmc_valid = "V";
//       return false;
//     }

//     // 00 .. 03
//     parsedRes.pps_status_index = buff[202];
//     // $GPRMC,,V,,,,,,,,,,N*53
//     std::stringstream isGprmcValid;
//     size_t dotCnt = 0;
//     for(size_t i = 210; i < 230; ++i) {
//       if(',' == buff[i]) {
//         ++dotCnt;
//       }
//       if(2 == dotCnt) {
//         isGprmcValid << buff[i + 1];
//         break;
//       }
//     }

//     // "3,A"
//     parsedRes.is_gprmc_valid = isGprmcValid.str();
//     return true;
// }



int main(int argc,char **argv) {
    int listenfd;
    sockaddr_in sockaddr;
    char buff[MAXLINE];

    bzero(&sockaddr, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr.sin_port = htons(8308);

    if((listenfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cout << "create socket error: " << strerror(errno) << "\n";
        exit(0);
    }

    int reuse = 1;
    if(setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cout << "Set sockopt error.";
        exit(0);
    }

    if(bind(listenfd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        std::cout << "bind socket error: " << strerror(errno) << "\n";
        close(listenfd);
        exit(0);
    }

    if (fcntl(listenfd, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
        std::cerr << "mSockfdPosition non-block\n";
        exit(0);
    }

    std::cout << "Please wait for the client information...\n";
    struct pollfd fds;
    fds.fd = listenfd;
    fds.events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)

    do {
        int retval = poll(&fds, 1, POLL_TIMEOUT);
        if(retval < 0) {
              if (errno != EINTR) {
                  std::cout << "poll() error: " << strerror(errno);
                  return 1;
              }
          }
        if(retval == 0) {
            std::cout << "Velodyne poll() timeout, go check IP configuration.\n";
            return 1;
        }
        if ((fds.revents & POLLERR) || (fds.revents & POLLHUP) || (fds.revents & POLLNVAL)) {
            std::cout << "poll() reports Velodyne error\n";
            return 1;
        }
    }
    while((fds.revents & POLLIN) == 0);

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);
    const size_t nbytes = recvfrom(listenfd, buff, POSITION_PACKET_SIZE, 0, (struct sockaddr *)&sender_address, &sender_address_len);

    close(listenfd);
    std::cout << nbytes << "\n";
    for(size_t i = 0; i < nbytes; ++i) {
        std::cout << std::hex << (int)buff[i];
    }
    std::cout << buff[206] << "\n";
    std::cout << std::hex << (int)buff[202] << "\n";

    size_t dotCnt = 0;
    for(size_t i = 210; i < 230; ++i) {
        if(',' == buff[i]) {
            ++dotCnt;
        }
        if(2 == dotCnt) {
            std::cout << buff[i + 1] << "\n";
            break;
        }
    }

}




