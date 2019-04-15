// 3 6
// 10 15 12
// 1 9 12 23 26 37

#include <iostream>
#include <deque>
#include <algorithm>

int main() {

    std::deque<int> dint {-888};
    std::cout << *min_element(dint.cbegin(), dint.cend()) << "\n";
}

