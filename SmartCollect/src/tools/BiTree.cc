#include <iostream>

struct BiNode {
    char data;
    BiNode *lchild;
    BiNode *rchild;
};


BiNode *CreatBiTree(BiNode *biNode, bool isRoot) {
    char ch;
    if (isRoot) {
        std::cout << "Root: \n";
    }
    std::cin >> ch;

    if ('#' == ch) {
        return biNode;
    }

    isRoot = false;
    biNode = new BiNode;
    biNode->data = ch;
    biNode->lchild = NULL;
    biNode->rchild = NULL;

    std::cout << biNode->data << "'s lchild: ";
    biNode->lchild = CreatBiTree(biNode->lchild, isRoot);
    std::cout << biNode->data << "'s rchild: ";
    biNode->rchild = CreatBiTree(biNode->rchild, isRoot);

    return biNode;
}


BiNode *CreatBiTreeIgnoreRoot(BiNode *biNode) {
    char ch;
    std::cin >> ch;

    if ('#' == ch) {
        goto cleanup;
    }

    biNode = new BiNode;
    biNode->data = ch;
    biNode->lchild = NULL;
    biNode->rchild = NULL;
    biNode->lchild = CreatBiTreeIgnoreRoot(biNode->lchild);
    biNode->rchild = CreatBiTreeIgnoreRoot(biNode->rchild);

cleanup:
    return biNode;
}

void PreOrder(BiNode *biTree) {
    if (!biTree) {
        return;
    }

    std::cout << biTree->data << "\n";
    PreOrder(biTree->lchild);
    PreOrder(biTree->rchild);
}

void InOrder(BiNode *biTree) {
    if (!biTree) {
        return;
    }

    InOrder(biTree->lchild);
    std::cout << biTree->data << "\n";
    InOrder(biTree->rchild);
}

void PostOrder(BiNode *biTree) {
    if (!biTree) {
        return;
    }

    PostOrder(biTree->lchild);
    PostOrder(biTree->rchild);
    std::cout << biTree->data << "\n";
}


BiNode *CreatBiTreeNoParam() {
    BiNode *biTree = NULL;
    char ch;
    std::cin >> ch;

    if ('#' == ch) {
        goto cleanup;
    }

    biTree = new BiNode;
    biTree->data = ch;
    biTree->lchild = CreatBiTreeNoParam();
    biTree->rchild = CreatBiTreeNoParam();

cleanup:
    return biTree;
}

int main() {
    // BiNode *biTree = CreatBiTree(biTree, true);
    Stacks S;

    BiNode *biTreeIgnoreRoot = NULL;
    biTreeIgnoreRoot = CreatBiTreeIgnoreRoot(biTreeIgnoreRoot);
    PreOrder(biTreeIgnoreRoot);

    BiNode *biTreeNoParam = CreatBiTreeNoParam();
    PreOrder(biTreeNoParam);

    return 0;
}

