#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdio.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
using namespace std;

int main(int argv,char *arg[]) {

    const std::string allFixedIndicator("/tmp/data_fixer_progress_100%");
    int i = remove(allFixedIndicator.c_str());
    cout << i << "\n";

    ofstream out(allFixedIndicator);
    if(out) {
        cout << "Success.\n";
    }
    else {
        cout << "Failed.\n";
    }
    cout << out << " end.\n";




    return 0;
}