#include <iostream>
#include <vector>

int main() {
    const int KEY = 8;
    std::vector<int> intArr {1, 2, 3, 4, 5, 9, 15, 57};

    if (KEY == intArr.front()) {
        std::cout << "0\n";

        return 0;
    }
    if (KEY == intArr.back()) {
        std::cout << intArr.size() - 1 << "\n";
        return intArr.size() - 1;
    }

    int bigIndex = intArr.size() - 1;
    int smallIndex = 0;
    int i = (bigIndex + smallIndex) / 2;

    while ( (i < bigIndex) && (i > smallIndex) ) {
        if (KEY > intArr[i]) {
            smallIndex = i;
        }
        else
        if (KEY < intArr[i]) {
            bigIndex = i;
        }
        else {
            std::cout << "i: " << i << "\n";
            return i;
        }
        i = (bigIndex + smallIndex) / 2;
    }

fail:
    std::cout << "Failed..\n";
    std::cout << "[" << smallIndex << ", " << bigIndex << "]\n";

    return 0;
}

