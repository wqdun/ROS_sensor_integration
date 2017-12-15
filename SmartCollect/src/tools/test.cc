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

template <typename T>
T string2num(const string& str, const T retIfEmpty) {
    if(str.empty() ) {
        cout << __FUNCTION__ << " param is null.";
        return retIfEmpty;
    }
    std::istringstream iss(str);
    T num;
    iss >> num;
    return num;
}

double weekSec2DaySec(const double weekSec) {
    // +8: Beijing Time
    double dayHour = (int)weekSec / 3600 % 24 + 8;
    double dayMinute = (int)weekSec / 60 % 60;
    double daySecond = fmod(weekSec, 3600 * 24);
    cout << daySecond << endl;
}

int main(int argc, char **argv) {
    cout << fixed << string2num("123.564", .0) << endl;
    // cout << weekSec2DaySec(279267.900) << endl;

}
