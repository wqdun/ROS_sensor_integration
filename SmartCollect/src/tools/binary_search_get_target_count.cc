#include <iostream>
#include <vector>

int BinarySearch(const std::vector<int> &inArr, int target) {
    std::cout << __FUNCTION__ << " start.\n";

    int left = 0;
    int right = inArr.size() - 1;

    while (left <= right) {
        int mid  = (left + right) / 2;
        if (target < inArr[mid]) {
            right = mid - 1;
        }
        else
        if (target > inArr[mid]) {
            left = mid + 1;
        }
        else {
            return mid;
        }
    }

    std::cout << "Failed.\n";
    return -1;
}

int GetSmallIndexOfTarget(const std::vector<int> &inArr, int target) {
    std::cout << __FUNCTION__ << " start.\n";

    int left = 0;
    int right = inArr.size() - 1;
    int result = -1;

    while (left <= right) {
        int mid  = (left + right) / 2;
        std::cout << "mid: " << mid << "\n";
        if (target < inArr[mid]) {
            right = mid - 1;
        }
        else
        if (target > inArr[mid]) {
            left = mid + 1;
        }
        else {
            result = mid;
            right = mid - 1;
        }
    }

    return result;
}

int GetBigIndexOfTarget(const std::vector<int> &inArr, int target) {
    std::cout << __FUNCTION__ << " start.\n";

    int left = 0;
    int right = inArr.size() - 1;
    int result = -1;

    while (left <= right) {
        int mid  = (left + right) / 2;
        std::cout << "mid: " << mid << "\n";
        if (target < inArr[mid]) {
            right = mid - 1;
        }
        else
        if (target > inArr[mid]) {
            left = mid + 1;
        }
        else {
            result = mid;
            left = mid + 1;
        }
    }

    return result;
}

int main(int argc, char const *argv[]) {
    std::cout << __FUNCTION__ << " start.\n";

    const int int2find = atoi(argv[1]);
    const std::vector<int> intArr {1, 8, 8, 8, 8, 10};

    const int smallIndex = GetSmallIndexOfTarget(intArr, int2find);
    const int bigIndex = GetBigIndexOfTarget(intArr, int2find);
    std::cout << bigIndex - smallIndex + 1 << " targets found.\n";

    return 0;
}

