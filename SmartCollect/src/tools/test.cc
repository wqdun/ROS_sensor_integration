#include <iostream>
#include <deque>
#include <algorithm>
#include <string>
#include <map>

using namespace std;



void sort_stdalg(deque<double> _dq) {
    std::sort(_dq.begin(), _dq.end());
}

void print_deque(deque<double> &_dq) {
    for(size_t i =0; i < _dq.size(); ++i) {
        std::cout << i << ": " << _dq[i] << "\n";
    }
}


int main(int argc, char const *argv[]) {
    std::deque<double> dq {1, 3, 2, 4, 0, 3.2, 3.6};
    std::deque<double> _dq(dq);
    print_deque(dq);
    sort_stdalg(dq);
    print_deque(dq);
    print_deque(_dq);
    std::cout << _dq[dq.size() / 2] << "\n";
    std::cout << dq[dq.size() / 2] << "\n";
    return 0;
}





