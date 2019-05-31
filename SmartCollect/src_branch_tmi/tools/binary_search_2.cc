#include <iostream>
#include <vector>

using namespace std;

int BinarySearch(const vector<int> &inArr, int target, int startIndex, int endIndex) {
    while (startIndex <= endIndex) {
        int midIndex = (startIndex + endIndex) / 2;

        if (target < inArr[midIndex]) {
            endIndex = midIndex - 1;
        }
        else
        if (target > inArr[midIndex]) {
            startIndex = midIndex + 1;
        }
        else {
            return midIndex;
        }
    }

    return -1;
}

int main(int argc, char const *argv[]) {
    vector<int> iArr {4, 4, 4, 1, 1};

    const int TARGET = atoi(argv[1]);
    if (iArr.empty()) {
        cout << "Failed.\n";
        return -1;
    }
    int bigIndex = iArr.size() - 1;
    int littleIndex = 0;
    int midIndex = (littleIndex + bigIndex) / 2;

    while (littleIndex <= bigIndex) {
        int midIndex = (littleIndex + bigIndex) / 2;

        if (TARGET == iArr[midIndex]) {
            cout << "Gotcha " << midIndex << "\n";
            return 0;
        }

        if (iArr[midIndex] > iArr[bigIndex]) {
            if ((TARGET >= iArr[littleIndex]) && (TARGET < iArr[midIndex])) {
                bigIndex = midIndex - 1;
            }
            else {
                littleIndex = midIndex + 1;
            }
        }
        // iArr[midIndex] <= iArr[bigIndex]
        else {
            if ((TARGET > iArr[midIndex]) && (TARGET <= iArr[bigIndex])) {
                littleIndex = midIndex + 1;
            }
            else {
                bigIndex = midIndex - 1;
            }
        }
    }

    cout << "Failed.\n";

    return 0;
}


