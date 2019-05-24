/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 */
#include <iostream>
#include <vector>

struct ListNode {
    int val;
    ListNode *next;
    ListNode(int x) : val(x), next(NULL) {}
};

using namespace std;
class Solution {
public:
    ListNode* addTwoNumbers(ListNode* l1, ListNode* l2) {
        std::vector<int> res {};
        int carry = 0;
        while (l1 || l2) {
            int addend1 = 0;
            int addend2 = 0;
            if (!l1) {
                addend1 = 0;
                addend2 = l2->val;
                l2 = l2->next;
            }
            else if (!l2) {
                addend1 = l1->val;
                addend2 = 0;
                l1 = l1->next;
            }
            else {
                addend1 = l1->val;
                addend2 = l2->val;
                l1 = l1->next;
                l2 = l2->next;
            }

            int sum = addend1 + addend2 + carry;
            if (sum > 9) {
                carry = 1;
                sum -= 10;
            }
            else {
                carry = 0;
            }

            res.push_back(sum);
        }

        if (carry) {
            res.push_back(1);
        }

        ListNode *ROOT = NULL;
        ListNode *list = NULL;
        for (int i = 0; i < res.size(); ++i) {
            if (0 == i) {
                ROOT = (list = new ListNode(res[0]));
                continue;
            }
            list->next = new ListNode(res[i]);
            list = list->next;
        }

        return ROOT;
    }


// public:
private:
    long long CalcReversedNumber(ListNode* list) {
        long long res = 0;
        int digitNum = 0;
        while (list) {
            res = (res * 10 + list->val);
            std::cout << list->val << "\t" << res << "\n";
            list = list->next;
            ++digitNum;
        }

        cout << res << "\t" << digitNum << "\n";
        // cout << GetReversedNum(res, digitNum) << "\n";
        return GetReversedNum(res, digitNum);
    }

    int GetDigitNum(long long inNum) {
        int digitNum = 0;
        for ( ; inNum > 10; ++digitNum) {
            inNum /=  10;
        }
        cout << digitNum + 1 << "\n";
        return digitNum + 1;
    }

    int GetDigit(long long num, int place) {
        int quotient = num;
        for (int i = 0; i < place; ++i) {
            quotient /=  10;
        }

        cout << quotient % 10 << "\n";
        return (quotient % 10);
    }

    long long GetReversedNum(long long inNum, int digitNum) {
        long long resReversed = 0;
        for (int i = 0; i < digitNum; ++i) {
            GetDigit(inNum, i);
            resReversed = (resReversed * 10 + GetDigit(inNum, i));
        }

        return resReversed;
    }

    ListNode *CreateListByNum(long long inNum) {
        ListNode *ROOT = NULL;
        ListNode *list = NULL;

        int digitNum = GetDigitNum(inNum);
        for (int i = 0; i < digitNum; ++i) {
            if (0 == i) {
                ROOT = (list = new ListNode(GetDigit(inNum, 0)));

            }
            else {
                list->next = new ListNode(GetDigit(inNum, i));
                list = list->next;
            }
        }

        return ROOT;
    }
};

int main() {
    std::vector<int> nums {6};
    ListNode *ROOT = NULL;
    ListNode *list = NULL;
    for (int i = 0; i < nums.size(); ++i) {
        if (0 == i) {
            ROOT = (list = new ListNode(nums[0]));
            continue;
        }
        list->next = new ListNode(nums[i]);
        list = list->next;
    }
    ListNode *list1 = ROOT;

    nums = {5};
    for (int i = 0; i < nums.size(); ++i) {
        if (0 == i) {
            ROOT = (list = new ListNode(nums[0]));
            continue;
        }
        list->next = new ListNode(nums[i]);
        list = list->next;
    }
    ListNode *list2 = ROOT;


    for (ListNode *l = ROOT; l != NULL; l = l->next) {
        cout << l->val << "\t";
    }

    return 0;
}