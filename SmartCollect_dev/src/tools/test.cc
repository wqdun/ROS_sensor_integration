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
    const vector<int> v1 {1, 2, 3, 4};
    vector<int> v2 {6, 7, 8, 9};

    // v1.insert(v1.end(), v2.begin(), v2.end() );
    // cout << v1.size() << endl;
    // for(size_t i = 0; i < v1.size(); ++i) {
    //     cout << v1[i] << " ";
    // }
    // cout << endl;

    std::move(v1.begin(), v1.end(), std::back_inserter(v2) );
    for(auto i: v2) {
        cout << i << " ";
    }
    cout << "\n";

    for(auto i: v1) {
        cout << i << " ";
    }

    cout << "\n";

}
