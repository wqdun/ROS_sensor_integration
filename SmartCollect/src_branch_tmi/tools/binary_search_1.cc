#include <iostream>
#include <vector>


using namespace std;
int main(int argc, char const *argv[]) {
    vector<int> iArr {1, 2, 4, 7};

    const int TARGET = atoi(argv[1]);
    if (iArr.empty()) {
        cout << "Failed.\n";
        return -1;
    }

    int bigIndex = iArr.size() - 1;
    int littleIndex = 0;

    if ((TARGET < iArr[littleIndex]) || (iArr[bigIndex] < TARGET)) {
        cout << "Failed.\n";
        return -1;
    }

    while (littleIndex <= bigIndex) {
        int midIndex = (littleIndex + bigIndex) / 2;
        if (TARGET < iArr[midIndex]) {
            bigIndex = midIndex - 1;
        }
        else
        if (TARGET == iArr[midIndex]) {
            cout << "Found in " << midIndex << "\n";
            return midIndex;
        }
        else {
            littleIndex = midIndex + 1;
        }
    }

    cout << "Not found.\n";
    return 0;
}


