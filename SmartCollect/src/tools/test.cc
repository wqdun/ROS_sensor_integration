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

double string2double(const string& str) {
    std::istringstream iss(str);
    double num;
    iss >> num;
    return num;
}
int main(int argc, char **argv) {

    cout << string2double("0") << endl;

}
