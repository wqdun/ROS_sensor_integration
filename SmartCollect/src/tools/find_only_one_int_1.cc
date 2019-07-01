#include <iostream>
#include <vector>

using namespace std;
int main() {
    const std::vector<int> intArr {1, 5, 2, 2, 5, 1, 9};
    if (intArr.empty()) {
        cout << "Failed.\n";
        return -1;
    }
    int res = intArr[0];
    for (int i = 1; i < intArr.size(); ++i) {
        res ^= intArr[i];
    }

    return res;
}


