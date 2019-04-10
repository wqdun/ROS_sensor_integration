#include <iostream>
#include <vector>

struct ListNode {
    int data;
    ListNode *next;

    ListNode(int x): data(x), next(NULL) {
    }
    ListNode(): data(0), next(NULL) {
    }
};

ListNode *CreatListNode() {
    ListNode *list = NULL;
    int inputInt;
    std::cin >> inputInt;
    if (-1 == inputInt) {
        std::cout << "Exit...\n";
        return list;
    }

    list = new ListNode(inputInt);
    list->next = CreatListNode();
    return list;
}

ListNode *CreatListNodebyVector(const std::vector<int> &inArr, size_t index) {
    ListNode *list = NULL;
    if (index == inArr.size()) {
        std::cout << "Exit...\n";
        return list;
    }

    list = new ListNode(inArr[index]);
    ++index;
    list->next = CreatListNodebyVector(inArr, index);
    return list;
}

void PutListNode(ListNode *list) {
    while (list) {
        std::cout << list->data;
        list = list->next;
    }

    std::cout << "\nEnd...\n";
}


int main() {
    // ListNode *listA = CreatListNode();
    // PutListNode(listA);

    std::vector<int> intArr {1, 2, 3, 4, 5, 6};
    size_t index = 0;
    ListNode *listB = CreatListNodebyVector(intArr, index);
    PutListNode(listB);

    std::vector<int> intArr2 {0, 8, 9};
    index = 0;
    ListNode *listC = CreatListNodebyVector(intArr2, index);
    PutListNode(listC);

    ListNode *mergedList = new ListNode();
    ListNode *temp = mergedList;
    while (listB && listC) {
        if (listC->data <= listB->data) {
            temp->next = listC;
            listC = listC->next;
        }
        else {

            temp->next = listB;
            listB = listB->next;
        }

        temp = temp->next;
    }

    temp->next = listB? listB: listC;

    PutListNode(mergedList->next);
    PutListNode(mergedList);
    return 0;
}

