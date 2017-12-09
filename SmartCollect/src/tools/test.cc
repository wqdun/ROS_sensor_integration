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
    char str1[] = "Hello";
    char c = '1';
    for(int i = 0; i < 5; ++i) {
        cout << hex << (int)str1[i] << endl; // or nread
    }

    char str2[5000] = "Hello";

    cout << strlen(str2) << endl;
    cout << dec << sizeof(str2) << endl;

    string str3(str2);
    printf("%c\n", str3[2]);
    cout << (str3[2] == '2');
    // printf("%s\n", str3);



}
