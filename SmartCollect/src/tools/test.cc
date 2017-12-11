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

using namespace std;

int main() {
    char cs[] = "Worl\0ddd";
    const string bufStr(cs);
    cout << bufStr << endl;
    cout << sizeof(int64_t) << endl;
}
