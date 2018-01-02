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

const short month[2][13] {
    365, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    366, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
};

bool isLeapYear(int year) {
    return (
        (0 == (year % 4) )
        && ( (0 != (year % 100) ) || (0 == (year % 4) ) )
    );
}

int main(int argc, char **argv) {




}
