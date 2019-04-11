#include <iostream>
#include <vector>


void swap(int &a, int &b) {
    int temp = a;
    a = b;
    b = temp;
}

int main() {
    std::vector<int> inArr {2, 4, 6, 3};

    const int inSize = inArr.size();
    for (int i = 0; i < inSize - 1; ++i) {
        std::cout << "i: " << i << "\n";
        int j = i + 1;

        int data2Sort = inArr[j];

        while ( (data2Sort < inArr[i]) && (i >= 0) ) {
            inArr[i + 1] = inArr[i];
            --i;
        }
        std::cout << "i: " << i << "; j: " << j << "\n";
        inArr[i + 1] = data2Sort;

        i = j - 1;
    }

    for (auto i: inArr) {
        std::cout << i << " ";
    }

    std::cout << "\n";
    return 0;
}

