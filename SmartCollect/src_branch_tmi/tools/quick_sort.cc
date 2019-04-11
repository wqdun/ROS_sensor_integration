#include <iostream>
#include <vector>


void QuickSort(int startIndex, int endIndex, std::vector<int> &inArr) {
    if ((endIndex - startIndex) <= 0) {
        return;
    }

    const int MID = inArr[startIndex];
    // int i = 0;
    int indexOfMid = startIndex;
    for (int i = 1 + startIndex; i <= endIndex; ++i) {
        if (inArr[i] < MID) {
            int temp = inArr[i];
            for (int j = i - 1; j >= startIndex; --j) {
                inArr[j + 1] = inArr[j];
            }
            ++indexOfMid;

            inArr[startIndex] = temp;
        }
    }
    std::cout << "indexOfMid: " << indexOfMid << "\n";
    QuickSort(startIndex, indexOfMid - 1, inArr);
    QuickSort(indexOfMid + 1, endIndex, inArr);
}

int main() {
    std::vector<int> inArr {5, 4, 2, 6, 1, -6, 8, 9, 4};

    QuickSort(0, inArr.size() - 1, inArr);
    for (auto i: inArr) {
        std::cout << i << " ";
    }

    std::cout << "\n";
    return 0;
}

