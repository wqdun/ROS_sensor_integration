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
// using namespace std;
using std::cout;
using std::endl;
using std::vector;


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
    vector<int> v1 {
        1, 2, 3, 4, 5,
    };

    vector<int> v2 {
        7, 8, 9, 10, 11,
    };

    // std::move(v1.begin(), v1.end(), std::back_inserter(v2) );

    std::move(v1.begin(), v1.end(), std::inserter(v2, v2.begin() ) );
    // v2 = v1.append(v2);
    // v1.insert(v1.end(), v2.begin(), v2.end() );
    // cout << v1.size() << endl;
    for(size_t i = 0; i < v2.size(); ++i) {
        cout << v2[i] << " ";
    }
    cout << endl;



}
