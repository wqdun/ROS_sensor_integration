#include <iostream>
#include <vector>

using namespace std;
class Solution {
public:
    int removeDuplicates(vector<int>& nums) {
        int resultLength = nums.size();
        for (int i = 0; i < resultLength; ++i) {
            for (int j = i + 1; j < resultLength; ++j) {
                if (nums[i] == nums[j]) {
                    Move1Step(nums, j);
                    --j;
                    --resultLength;
                }
                else {
                    break;
                }
            }
        }

        return resultLength;
    }


private:
    void Move1Step(vector<int>& arrToMove, int index) {
        for (int i = index + 1; i < arrToMove.size(); ++i) {
            arrToMove[i - 1] = arrToMove[i];
        }
    }

};

int main(int argc, char const *argv[]) {
    Solution sol;
    vector<int> nums {0,0,1,1,1,2,2,3,3,4};

    if (nums.size() <= 1) {
        return nums.size();
    }

    for (auto it = nums.begin() + 1; it != nums.end(); ++it) {
        if ((*(it - 1)) == (*(it))) {
            it = nums.erase(it);
        }
    }

    for (auto i: nums) {
        cout << i << " ";
    }

    // cout << resultLength << "\n";


    return 0;
}

