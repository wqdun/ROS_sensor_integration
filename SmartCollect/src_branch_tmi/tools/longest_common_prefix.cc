#include <iostream>
#include <string>
#include <vector>



using namespace std;
class Solution {
public:
    string longestCommonPrefix(vector<string>& strs) {
        if (strs.empty()) {
            return "";
        }
        
        const int shortestLength = GetShortestLength(strs);
        int longestIndex = shortestLength;
        for (int i = 0; i < shortestLength; ++i) {
            if (!IsCharSame(strs, i)) {
                longestIndex = i;
                break;
            }
        }
        
        return strs[0].substr(0, longestIndex);

    }


// private:
public:
    int GetShortestLength(const vector<string>& strs) {
        int ans = -1;
        for (auto str: strs) {
            if (str.size() < ans) {
                ans = str.size();
            }
        }
        return ans;
    }

    bool IsCharSame(const vector<string>& strs, int index) {
        for (int i = 0; i < strs.size(); ++i) {
            for (int j = i + 1; j < strs.size(); ++j) {
                if (strs[i][index] != strs[j][index]) {
                    return false;
                }
            }
        }
        return true;
    }
};



int main(int argc, char const *argv[]) {
    Solution sol;

    vector<string> strs {
        "hello",
        "horld",
        "hi",
        "h"
    };

    cout << sol.longestCommonPrefix(strs) << "\n";

    return 0;
}

