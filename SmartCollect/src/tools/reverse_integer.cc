#include <iostream>
#include <vector>
#include <climits>

using namespace std;
class Solution {
public:
    int reverse(int x) {
        cout << "INT_MAX: " << INT_MAX << "\n";
        cout << "INT_MIN: " << INT_MIN << "\n";
        if (x > INT_MAX || x <= INT_MIN || 0 == x) {
            return 0;
        }


        bool isPositive = (x >= 0);
        if (!isPositive) {
            x = -x;
        }
        std::vector<int> res {};
        while (x) {
            int digit = x % 10;
            res.push_back(digit);
            x /= 10;
        }

        long ans = 0;
        for (auto i: res) {
            cout << i << "\n";
            ans = 10 * ans + i;
        }
        cout << "\n";

        if (ans > INT_MAX || ans < INT_MIN ) {
            return 0;
        }

        if (!isPositive) {
            ans = -ans;
        }

        return ans;
    }
};




int main(int argc, char const *argv[]) {
    Solution solution;

    cout << solution.reverse(-120) << "\n";

    return 0;
}




