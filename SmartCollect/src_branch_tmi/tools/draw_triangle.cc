#include <iostream>

int main() {
    int a[30][50];
    for (int i = 0; i < 30; ++i) {
        for (int j = 0; j < 50; ++j) {
            if ((3 * i) < (5 * j)) {
                a[i][j] = 1;
            }
            else {
                a[i][j] = 0;
            }
        }
    }

    for (int i = 0; i < 30; ++i) {
        for (int j = 0; j < 50; ++j) {
            if (0 == a[i][j]) {
                std::cout << "*";
            }
            else {
                // std::cout << " ";
            }

        }
        std::cout << "\n";
    }



    return 0;
}

