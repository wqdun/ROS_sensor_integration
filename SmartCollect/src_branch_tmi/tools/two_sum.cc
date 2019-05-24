#include <iostream>
#include <vector>
#include <map>

using namespace std;
class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        std::multimap<int, int> num_indexMap;
        for (int i = 0; i < nums.size(); ++i) {
            num_indexMap.insert({nums[i], i});
        }
        std::cout << num_indexMap.size() << "\n";

        int index = -1;
        for (const auto &num_index: num_indexMap) {
            ++index;
            int numToFind = target - num_index.first;
            int indexRes = FindNumInMap(numToFind, num_indexMap, index + 1);
            if (indexRes < 0) {
                continue;
            }
            std::cout << num_index.second << ", " << indexRes << "\n";
            if (num_index.second > indexRes) {
                return std::vector<int> {indexRes, num_index.second};
            }
            return std::vector<int> {num_index.second, indexRes};
        }

        std::cout << "Failed to find.\n";
        return std::vector<int> {};
    }


private:
    int FindNumInMap(int numToFind, const std::multimap<int, int> &num_indexMap, int startIndex) {
        int index = -1;
        for (const auto &num_index: num_indexMap) {
            ++index;
            if (index < startIndex) {
                continue;
            }

            if (numToFind < num_index.first) {
                return -1;
            }
            else
            if (numToFind == num_index.first) {
                return num_index.second;
            }
            // else {
            //     continue;
            // }
        }

        return -1;
    }

};