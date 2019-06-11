#include <iostream>
#include <vector>

using namespace std;
int sum(vector<int> &inArr, int num) {
    if (0 == num) {
        return inArr[0];
    }

    int res = inArr[num] + sum(inArr, num - 1);
    return res;
}

int main(int argc, char const *argv[]) {
    int input = atoi(argv[1]);
    vector<int> arr {2, 3, 10};

    cout << sum(arr, input) << "\n";
    return 0;
}

