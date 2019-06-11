#include <iostream>
#include <vector>

using namespace std;
int BinarySearch(vector<int> &arr, int target) {
    int smallEnd = 0;
    int bigEnd = arr.size() - 1;
    if ((target < arr[smallEnd]) || (target > arr[bigEnd])) {
        cout << "Not found.\n";
        return -1;
    }

    while (smallEnd <= bigEnd) {
        int mid = (smallEnd + bigEnd) / 2;

        if (target < arr[mid]) {
            bigEnd = mid - 1;
        }
        else
        if (target == arr[mid]) {
            cout << "Found in " << mid << "\n";
            return mid;
        }
        // target > arr[mid]
        else {
            smallEnd = mid + 1;
        }
    }

    cout << "Not found.\n";
    return -2;
}


int BinarySearchRecursion(vector<int> &arr, int target, int smallEnd, int bigEnd) {
    // cout << "smallEnd: " << smallEnd << "\n";
    // cout << "bigEnd: " << bigEnd << "\n";

    if ((target < arr[smallEnd]) || (target > arr[bigEnd])) {
        cout << "Failed.\n";
        return -1;
    }

    if (smallEnd > bigEnd) {
        cout << "Failed.\n";
        return -2;
    }

    int mid = (smallEnd + bigEnd) / 2;
    // cout << "mid: " << mid << "\n";

    if (target < arr[mid]) {
        bigEnd = mid - 1;
    }
    else
    if (target == arr[mid]) {
        cout << "Found in " << mid << "\n";
        return mid;
    }
    // target > arr[mid]
    else {
        smallEnd = mid + 1;
    }

    return BinarySearchRecursion(arr, target, smallEnd, bigEnd);
}

BinarySearchRecursion_if(vector<int> &arr, int target) {
    return BinarySearchRecursion(arr, target, 0, arr.size() - 1);
}



int main(int argc, char const *argv[]) {
    vector<int> inArr {1, 3, 6, 7};
    int target = atoi(argv[1]);
    cout << "target: " << target << "\n";
    cout << BinarySearch(inArr, target) << "\n";
    cout << BinarySearchRecursion_if(inArr, target) << "\n";

    return 0;
}

