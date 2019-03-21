#include <iostream>
#include <stdio.h>
#include <cstdlib>
// pack declaration should be placed before struct declaration
#pragma pack(2)


using namespace std;

typedef struct {
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


    bool isValid = true;
    pktInfo_t pktInfo;
    size_t pos = 0;
    size_t readLen = 0;
    double time1 = -1;
    double time2 = -1;
    double timeErr = -1;
    cout << std::fixed;
    // 循环读取文件
    do {
        readLen = fread(&pktInfo, pktSize, 1, pInFile);
        // cout << readLen << endl;
        if(0 >= readLen) {
            cout << "[ERROR] Packet[" << pos / pktSize << "] is not a complete packet.\n";
            isValid = false;
            // exit(-1);
        }

        pos += pktSize;

        // assert(pktInfo.data.size() >= 1204);
        const unsigned int pktTimeHourSInUs = (pktInfo.data[1203] << 24) + (pktInfo.data[1202] << 16) + (pktInfo.data[1201] << 8) + pktInfo.data[1200];
        // cout << pktTimeHourSInUs / 1000000. << endl;

        time1 = time2;
        time2 = pktInfo.timeStamp;
        timeErr = time2 - time1;
        // cout << "timeErr:" << timeErr << "; time2:" << time2 << "; time1: " << time1 << "\n";
        if(0 > time1) {
            continue;
        }
        if(timeErr > 0.0015) {
            cout << "[ERROR] Packet[" << pos / pktSize << "] time interval:" << timeErr <<
            "; time1:" << time1 << "; time2:" << time2 << endl;
            cout << "pktTimeHourSInUs: " << pktTimeHourSInUs / 1000000. << endl;
            isValid = false;
            // exit(-1);
        }
        if(timeErr < -1) {
            cout << "Record ends here:" << time1 << "; start:" << time2 << endl;
            cout << "pktTimeHourSInUs: " << pktTimeHourSInUs / 1000000. << endl;
        }

    } while(pos < fileSize);

    if(isValid) {
        cout << argv[1] << " is a valid lidar data.\n";
    }
    else {
        cout << argv[1] << " is not valid.\n";
    }

    // 释放资源
    fclose(pInFile);

    return 0;
}

