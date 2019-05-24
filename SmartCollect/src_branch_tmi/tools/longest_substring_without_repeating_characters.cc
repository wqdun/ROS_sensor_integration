#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
class Solution {
public:
    int lengthOfLongestSubstring(string s) {
        std::vector<int> res {0};
        for (int i = 0; i < s.size(); ++i) {
            // cout << s.substr(i) << "\n";
            res.push_back(GetLongestSubstringWithoutRepeatingCharactersFrom0_2(s.substr(i)));
        }

        cout << (*std::max_element(res.cbegin(), res.cend())) << "\n";
        cout << res.size() << "\n";
        // for (auto i: res) {
        //     cout << i << "\t";
        // }
        cout << "\n";
        return (*std::max_element(res.cbegin(), res.cend()));

    }


// public:
private:
    bool IsDuplicated(const std::string &str, char c) {
        for (auto ch: str) {
            if (ch == c) {
                return true;
            }
        }

        return false;
    }

    bool IsDuplicated_2(const std::string &str, char c) {
        if (str.find(c) != string::npos) {
            return true;
        }

        return false;
    }


    int GetLongestSubstringWithoutRepeatingCharactersFrom0(const std::string &str) {
        std::string subStr("");
        for (int i = 0; i < (str.size() - 1); ++i) {
            subStr += str[i];
            if (IsDuplicated(subStr, str[i + 1])) {
                return subStr.size();
            }
        }
        return str.size();
    }


    int GetLongestSubstringWithoutRepeatingCharactersFrom0_2(const std::string &str) {
        // std::vector<int> hash(1000, 0);
        int hash[200] = {0};
        int result = -1;

        for (int i = 0; i < str.size(); ++i) {
            ++result;
            ++hash[str[i]];
            if (2 == hash[str[i]]) {
                // std::vector<int>().swap(hash);
                return result;
            }
        }
        // std::vector<int>().swap(hash);
        return str.size();
    }

    int GetLongestSubstringWithoutRepeatingCharactersFrom0_3(const std::string &str) {
        std::string subStr("");
        for (int i = 0; i < (str.size() - 1); ++i) {
            subStr += str[i];
            if (IsDuplicated_2(subStr, str[i + 1])) {
                return subStr.size();
            }
        }
        return str.size();
    }
};






int main() {
    Solution solution;
    cout << solution.lengthOfLongestSubstring("dd") << "\n";
    // cout << solution.GetLongestSubstringWithoutRepeatingCharactersFrom0_2("abcb") << "\n";



}

