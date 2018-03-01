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
    string tmp("temp string");
    string tmp2("hi", 7);
    cout << tmp2 << endl;

    vector<string> strs;
    strs.push_back(tmp);
    strs.push_back("tmp");
    strs.emplace_back("hello");
    strs.emplace_back(string("hello"));

    for(auto &i: strs) {
        cout << i << endl;
    }


    return 1;
}