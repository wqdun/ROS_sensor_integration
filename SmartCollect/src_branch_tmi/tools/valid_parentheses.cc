#include <iostream>
#include <string>
#include <vector>

using namespace std;
class Solution {
public:
    bool isValid(string s) {
        vector<char> charArr {0};
        for (auto c: s) {
            char peek = charArr.back();
            if (IsOpposite(peek, c)) {
                charArr.pop_back();
            }
            else {
                charArr.push_back(c);
            }
        }

        return (1 == charArr.size());
    }

// public:
private:
    bool IsOpposite(char a, char b) {
        if (a == '(') {
            return (b == ')');
        }

        if (a == '[') {
            return (b == ']');
        }

        if (a == '{') {
            return (b == '}');
        }

        return false;
    }

};


int main(int argc, char const *argv[]) {
    Solution sol;
    cout << sol.isValid("();") << "\n";

    return 0;
}

