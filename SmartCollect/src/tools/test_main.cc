#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>

#define LOG(ERROR) std::cout



int popenWithReturn(const std::string &cmd, std::vector<std::string> &cmdReturn) {
    const size_t maxByte = 1000;
    char result[maxByte];
    FILE *fpin;

    if(NULL == (fpin = popen(cmd.c_str(), "r") ) ) {
        LOG(ERROR) << "Failed to open " << cmd;
        return -1;
    }

    while(fgets(result, maxByte, fpin) ) {
        cmdReturn.push_back(result);
    }

    if(0 != pclose(fpin) ) {
        LOG(WARNING) << "Failed to close " << cmd;
        return -2;
    }

    return 0;
}


int main()
{
    const std::string lscmd("ls");

    std::vector<std::string> reses;
    popenWithReturn(lscmd, reses);

    for(auto &res: reses) {
        std::cout << res << "\n";
    }


    return 0;
}



