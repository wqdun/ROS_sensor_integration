#include <iostream>
#include <vector>


struct NodeList {
    int data;
    NodeList *next;

    NodeList(): data(0), next(NULL) {
    }
    NodeList(int i): data(i), next(NULL) {
    }
};


int GetMinIndex(const std::vector<int> &inArr, int startIndex, int endIndex) {
    int minIndex = startIndex;
    for (int i = startIndex + 1; i <= endIndex; ++i) {
        if (inArr[i] < inArr[minIndex]) {
            minIndex = i;
        }
    }

    return minIndex;
}

void swap(int &a, int &b) {
    int temp = a;
    a = b;
    b = temp;
}

NodeList *ReverseList(NodeList *pHead) {
    NodeList *pList = pHead;

    NodeList *previous = NULL;
    while (pList) {
        NodeList *next = pList->next;
        pList->next = previous;
        previous = pList;

        pList = next;
    }

    return previous;
}


int main() {
    std::vector<int> inArr {5, 4, 2, 6, 1, -6, 8, 9, 4};

    NodeList *pNodeList = NULL;

    pNodeList = new NodeList();
    NodeList *ROOT = pNodeList;
    for (int i = 0; i < inArr.size(); ++i) {
        pNodeList->next = new NodeList(inArr[i]);
        pNodeList = pNodeList->next;
    }

    pNodeList = ROOT->next;
    while (pNodeList) {
        std::cout << pNodeList->data << " ";
        pNodeList = pNodeList->next;
    }
    std::cout << "\n";

    NodeList *newRoot = ReverseList(ROOT);
    pNodeList = newRoot->next;
    while (pNodeList) {
        std::cout << pNodeList->data << " ";
        pNodeList = pNodeList->next;
    }

    std::cout << "\n";
    return 0;
}

