#include <iostream>

class DynamicVector
{
public:
    DynamicVector();
    ~DynamicVector();



private:
    int size_;
    int *pVec_;
};

DynamicVector::DynamicVector() {
    pVec_ = new int[10];
    capacity_ = 10;
    size_ = 0;
}




void DynamicVector::push_back(int num) {
    ++size_;
    if (size_ > capacity_) {

    }




}



int main(int argc, char const *argv[])
{
    /* code */
    return 0;
}
