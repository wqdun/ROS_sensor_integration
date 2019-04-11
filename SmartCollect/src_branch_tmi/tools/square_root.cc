#include <iostream>
#include <vector>

double GetSquare(double in) {
    return in * in;
}

int main() {
    const int KEY = 7;

    double big = (KEY + 1) / 2;
    double small = 0;
    while (small < (big - 0.0000000001) ) {
        double mid = (small + big) / 2;
        std::cout << "mid: " << mid << ": [" << small << ", " << big << "]\n";
        if (KEY > GetSquare(mid)) {
            small = mid;
        }
        else
        if (KEY < GetSquare(mid)) {
            big = mid;
        }
        else {
            std::cout << "mid: " << mid << "\n";
            return mid;
        }
    }

fail:
    std::cout << "Failed..\n";
    std::cout << "[" << small << ", " << big << "]\n";

    return 0;
}

