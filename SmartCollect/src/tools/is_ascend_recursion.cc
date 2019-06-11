#include <iostream>
#include <vector>

using namespace std;


bool IsAscend(const vector<int> &arr, int n) {
    if (n <= 0) {
        return true;
    }

    if ((arr[n] > arr[n - 1]) && (IsAscend(arr, n - 1))) {
        return true;
    }
    else {
        return false;
    }
}



int main(int argc, char const *argv[]) {
    vector<int> arr {1, 2, 1};
    cout << std::boolalpha << IsAscend(arr, arr.size() - 1);
    return 0;
}




