#include <iostream>
#include <assert.h>
#include <math.h>

int main() {
    const int IN = 5;
    assert(IN >= 2);
    // if (IN < 4) {
    //     std::cout << IN << ": Yes.\n";
    //     return 0;
    // }
    std::cout << sqrt(IN) << "\n";
    for (int i = 2; i <= sqrt(IN); ++i) {
        if (0 == IN % i) {
            std::cout << i << ": No.\n";
            return 0;
        }
    }

    std::cout << IN << " Yes.\n";
    return 0;
}

