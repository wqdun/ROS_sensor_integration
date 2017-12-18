#include <iostream>
#include <sstream>
#include <cmath>
#include <bitset>
#include <string>
#include <algorithm>
#include <cstdio>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <pthread.h>
#include <unistd.h>
using namespace std;

pthread_t ntid;
int g_i = 0;

void *thread1(void *arg) {
    while(1) {
        cout << "thread1" << endl;
        ++g_i;
        usleep(500000);

    }
    // ++g_i;
}

int main(int argc, char **argv) {
    int err = pthread_create(&ntid, NULL, thread1, NULL);
    if(0 != err) {
        return 1;
    }
    while(1) {
        for(int i = 0; i < g_i; ++i) {
            cout << i << endl;
            usleep(100000);
            cout << ":g_i:" << g_i << endl;
        }
        usleep(100000);
    }
    return 0;
}
