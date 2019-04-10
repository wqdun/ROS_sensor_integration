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

int main() {
    std::vector<int> inArr {5, 4, 2, 6, 1, -6, 8, 9, 4};

    for (int start = 0; start < inArr.size(); ++start) {
        int minIndex = GetMinIndex(inArr, start, inArr.size() - 1);
        if (minIndex != start) {
            swap(inArr[start], inArr[minIndex]);
        }
    }


    for (int i = 0; i < inArr.size(); ++i) {
        std::cout << inArr[i] << " ";
    }


    // NodeList *pNodeList = NULL;

    // pNodeList = new NodeList();
    // NodeList *ROOT = pNodeList;
    // for (int i = 0; i < inArr.size(); ++i) {
    //     pNodeList->next = new NodeList(inArr[i]);
    //     pNodeList = pNodeList->next;
    // }

    // pNodeList = ROOT;
    // while (pNodeList) {
    //     std::cout << pNodeList->data << " ";
    //     pNodeList = pNodeList->next;
    // }

    std::cout << "\n";
    return 0;
}

