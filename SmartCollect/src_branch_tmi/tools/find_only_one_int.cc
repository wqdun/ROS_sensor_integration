#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

using namespace std;
// class Solution {
// public:



// // public:
// private:

// };


int main() {
    std::vector<int> intArr {1, 2, 3, 5, 3, 5, 8, 0, -5};

    const int MIN = *(std::min_element(intArr.cbegin(), intArr.cend()));
    const int MAX = *(std::max_element(intArr.cbegin(), intArr.cend()));

    const int SIZE = MAX - MIN + 1;
    std::vector<int> hashTable(SIZE, 0);
    cout << hashTable.size() << "\n";
    for (auto i: intArr) {
        ++hashTable[i - MIN];
    }

    for (int i = 0; i < hashTable.size(); ++i) {
        if (1 == hashTable[i]) {
            cout << i + MIN << " ";
        }
    }

    cout << "\n";

    std::vector<double> doubleArr {1.3, 2.2, 2.3, 2.2, 3.6, 5, 8, 0.1, 0, -5.3};
    std::map<double, int> val_countMap;
    cout << val_countMap.size() << "\n";
    val_countMap[77];
    cout << val_countMap.size() << " " << val_countMap[77];

    // for (auto i: doubleArr) {
    //     ++val_countMap[i];
    // }

    // for (auto val_count: val_countMap) {
    //     // cout << val_count.first << " " << val_count.second << "\n";
    //     if (1 == val_count.second) {
    //         cout << val_count.first << "\t";
    //     }
    // }


    cout << "\n";
    return 0;
}