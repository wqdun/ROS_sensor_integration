#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

int main(int argv,char *arg[]) {

    std::vector<std::string> words;
    // words.push_back("gg");
    for(auto &word: words) {
        std::cout << word << "\n";
    }

    return 0;
}