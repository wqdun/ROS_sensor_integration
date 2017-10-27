#include <iostream>
#include <stdio.h>
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
    cout << fileSize << "\n";
    fseek(pInFile, 0, SEEK_SET);
    const size_t pktSize = sizeof(pktInfo_t);
    cout << "pktSize:" << pktSize << "\n";
    cout << argv[1] << " contains " << fileSize / pktSize << " packets.\n";

    pktInfo_t pktInfo;
    size_t pos = 0;
    size_t readLen = 0;
    double time1 = 0;
    double time2 = 0;
    // 循环读取文件
    do {
        readLen = fread(&pktInfo, pktSize, 1, pInFile);
        if(readLen > 0) {
            pos += pktSize;
            cout << readLen << "\n";
            // 对读取的文件做处理
            cout << fixed << pktInfo.timeStamp << "\n";

            time1 = time2;
            time2 = pktInfo.timeStamp;
            cout << "timeErr = " << time2 - time1 << "\n";
        }
        else {
            cout << "It is null.\n";
        }
    } while(pos < fileSize);

    // 释放资源
    fclose(pInFile);
    return 0;
}

