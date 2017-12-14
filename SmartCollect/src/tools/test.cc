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

typedef struct {
    double d;
    vector<int> i;
} vec_t;

typedef struct {
    double d;
    int i[40];
} vec2_t;

int main(int argc, char **argv) {
    vec_t vec;
    vec.d = 4;
    vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);
    vec.i.push_back(2);
        vec.i.push_back(1);


    cout << vec.i[0] << endl;
    cout << sizeof(vec.i) << endl;
    cout << sizeof(vec2_t) << endl;

}
