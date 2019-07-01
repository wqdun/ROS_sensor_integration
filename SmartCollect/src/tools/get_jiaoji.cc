#include <iostream>
#include <vector>
#include <map>

using namespace std;

vector<int> GetJiaoJi(const vector<int>& a1, const vector<int>& a2) {
    vector<int> result {};

    int i = 0;
    int j = 0;

    while ((i < a1.size()) && (j < a2.size())) {
        cout << "i " << i << "; j: " << j << "\n";
        if (a1[i] < a2[j]) {
            ++i;
        }
        else
        if (a1[i] > a2[j]) {
            ++j;
        }
        else {
            if ((result.empty()) || (result.back() != a1[i])) {
                result.push_back(a1[i]);
            }
            ++i;
            ++j;
        }
    }

    return result;
}

vector<int> GetJiaoJi_Hashtable(const vector<int>& a1, const vector<int>& a2) {
    vector<int> result {};

    map<int, int> hashTab {};

    for (auto i: a1) {
        ++hashTab[i];
    }
    for (auto j: a2) {
        ++hashTab[j];
    }

    for (auto k: hashTab) {
        if (k.second > 1) {
            cout << k.first << "\t";
            cout << k.second << "\t";
            cout << "\n";
            result.push_back(k.first);
        }
    }
    return result;
}

vector<int> GetJiaoJi_Hashtable(const vector<int>& a1, const vector<int>& a2) {
    vector<int> result {};

    for (auto i: a1) {

    }

    for (auto k: hashTab) {
        if (k.second > 1) {
            cout << k.first << "\t";
            cout << k.second << "\t";
            cout << "\n";
            result.push_back(k.first);
        }
    }
    return result;
}



int main(int argc, char const *argv[]) {
    const vector<int> arr1 {1, 2, 3, 3, 4, 5};
    const vector<int> arr2 {1, 3, 3, 5, 7, 9};

    const vector<int> res(GetJiaoJi_Hashtable(arr1, arr2));

    for (auto i: res) {
        cout << i << "\t";
    }
    cout << "\n";

    return 0;
}




