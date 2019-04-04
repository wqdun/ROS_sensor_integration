#include <iostream>
#include <sstream>

class AdditionCycles {
public:
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

    int cycleLength(int inNum) {
        const int origin = inNum;
        int _inNum = origin;

        int counter = 0;
        while (true) {
            ++counter;
            int digitSum = GetDigitSum(_inNum);
            _inNum = 10 * Get2nd(_inNum) + Get2nd(digitSum);
            // std::cout << _inNum << "\n";
            if (_inNum == origin) {
                std::cout << counter << "\n";
                return counter;
            }
        }

        // std::cout << "Bye...\n";
        return -1;
    }

}




