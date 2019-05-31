#include <iostream>
#include <vector>
#include <map>
#include <string>

// Symbol       Value
// I             1
// V             5
// X             10
// L             50
// C             100
// D             500
// M             1000

using namespace std;
class Solution {
public:
    int romanToInt(string s) {
        std::map<char, int> romanDigit_intMap {
            {'I', 1},
            {'V', 5},
            {'X', 10},
            {'L', 50},
            {'C', 100},
            {'D', 500},
            {'M', 1000}
        };

        int sum = 0;
        for (int i = 0; i < s.size() - 1; ++i) {
            int num0 = romanDigit_intMap[s[i]];
            int num1 = romanDigit_intMap[s[i + 1]];
            if (num0 < num1) {
                sum -= num0;
            }
            else {
                sum += num0;
            }

        }

        sum += romanDigit_intMap[s.back()];
        return sum;
    }

};

int main() {
    std::vector<int> intArr {1, 2, 3, 5, 3, 5, 8, 0, -5};

    std::string s("IIV");
    std::map<char, int> romanDigit_intMap {
        {'I', 1},
        {'V', 5},
        {'X', 10},
        {'L', 50},
        {'C', 100},
        {'D', 500},
        {'M', 1000}
    };

    int sum = 0;
    for (int i = 0; i < s.size() - 1; ++i) {
        // cout << romanDigit_intMap[s[i]] << " ";
        int num0 = romanDigit_intMap[s[i]];
        int num1 = romanDigit_intMap[s[i + 1]];
        if (num0 < num1) {
            sum -= num0;
        }
        else {
            sum += num0;
        }

    }

    sum += romanDigit_intMap[s.back()];

    cout << sum << "\n";
    Solution sol;
    cout << sol.romanToInt("IIV") << "\n";
    return 0;
}