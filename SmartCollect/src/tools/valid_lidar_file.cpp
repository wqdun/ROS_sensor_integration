#include <iostream>
#include <stdio.h>
#include <cstdlib>
#pragma pack(2)

using namespace std;

typedef struct pktInfo {
    double timeStamp;
    unsigned char data[1206];
  } pktInfo_t;


int main(int argc, char **argv) {
    if(2 != argc) {
        cout << "Param error.\n";
        return -1;
    }

    FILE *pInFile = fopen(argv[1], "rb");
    if(!pInFile) {
        cout << "Open " << argv[1] << " error.\n";
        return -1;
    }

    fseek(pInFile, 0, SEEK_END);
    long fileSize = ftell(pInFile);
    cout << "fileSize:" << fileSize << "\n";
    fseek(pInFile, 0, SEEK_SET);
    const size_t pktSize = sizeof(pktInfo_t);
    cout << "pktSize:" << pktSize << "\n";
    const size_t pktCnt = fileSize / pktSize;
    cout << argv[1] << " contains " << fileSize / pktSize << " packets.\n";

    pktInfo_t pktInfo;
    size_t pos = 0;
    size_t readLen = 0;
    double time1 = -1;
    double time2 = -1;
    double timeErr = -1;
    // 循环读取文件
    do {
        readLen = fread(&pktInfo, pktSize, 1, pInFile);
        // cout << readLen << endl;
        if(0 >= readLen) {
            cout << "[ERROR] Packet[" << pos / pktSize <<  "] is not a complete packet.\n";
            exit(-1);
        }

        pos += pktSize;

        // cout << fixed << pktInfo.timeStamp << "\n";

        time1 = time2;
        time2 = pktInfo.timeStamp;
        timeErr = time2 - time1;
        // cout << "timeErr = " << timeErr << "\n";
        if(0 > time1) {
            continue;
        }
        if(timeErr > 0.0015) {
            cout << "[ERROR] Might lost some packets, time interval:" << timeErr <<
            "; time1:" << time1 << "; time2:" << time2 << endl;
            exit(-1);
        }

    } while(pos < fileSize);

    cout << argv[1] << " is a valid lidar data.\n";

    // 释放资源
    fclose(pInFile);

    return 0;
}

