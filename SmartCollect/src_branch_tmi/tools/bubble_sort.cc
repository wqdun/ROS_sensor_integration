#include <iostream>
#include <vector>


void swap(int &a, int &b) {
    int temp = a;
    a = b;
    b = temp;
}


int main() {
    std::vector<int> inArr {1, 3, 2, 0, 9, 45, 10, -4};

    const int inSize = inArr.size();
    for (int i = 0; i < inSize; ++i) {
        for (int j = i + 1; j < inSize; ++j) {
            if (inArr[i] > inArr[j]) {
                swap(inArr[i], inArr[j]);
            }
        }
    }

    // for (auto i = inArr.begin(); i != inArr.end(); ++i) {
    //     std::cout << *i << " ";
    // }

    for (auto i: inArr) {
        std::cout << i << " ";
    }

    std::cout << "\n";
    return 0;
}

