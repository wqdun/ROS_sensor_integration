#include <iostream>
#include <sstream>

int Get1st(int inNum) {
    return inNum / 10;
}

int Get2nd(int inNum) {
    return inNum % 10;
}


int GetDigitSum(int inNum) {
    return Get1st(inNum) + Get2nd(inNum);
}

int String2Int(const std::string& str) {
    if(str.empty() ) {
        return -1;
    }
    std::istringstream iss(str);
    int num;
    iss >> num;
    return num;
}

int main(int argc, char *argv[]) {
    const int origin = String2Int(argv[1]);
    int inNum = origin;

    int counter = 0;
    while (true) {
        ++counter;
        int digitSum = GetDigitSum(inNum);
        inNum = 10 * Get2nd(inNum) + Get2nd(digitSum);
        // std::cout << inNum << "\n";
        if (inNum == origin) {
            std::cout << counter << "\n";
            return counter;
        }
    }

    // std::cout << "Bye...\n";
    return -1;
}

