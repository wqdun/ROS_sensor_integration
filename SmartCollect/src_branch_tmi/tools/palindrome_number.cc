#include <iostream>
#include <vector>

using namespace std;
class Solution {
public:
    bool isPalindrome(int x) {
        if (x < 0) {
            return false;
        }
        std::vector<int> arr = GetVectorByInt(x);
        return IsPalindrome(arr);
    }


private:
// public:
    std::vector<int> GetVectorByInt(int x) {
        std::vector<int> res {};
        while (x) {
            int digit = x % 10;
            x /= 10;
            res.push_back(digit);
        }

        for (auto i: res) {
            cout << i << " ";
        }
        cout << "\n";
        return res;
    }

    bool IsPalindrome(const std::vector<int> &digitArr) {
        for (int i = 0; i < digitArr.size() / 2; ++i) {
            if (digitArr[i] != digitArr[digitArr.size() - i - 1]) {
                return false;
            }
        }
        return true;
    }


};




int main(int argc, char const *argv[]) {
    Solution solution;

    cout << solution.isPalindrome(10) << "\n";

    return 0;
}


