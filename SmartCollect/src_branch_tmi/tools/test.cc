#include <iostream>     // std::cout
#include <algorithm>    // std::remove_if
#include <vector>

bool IsOdd (int i) { return ((i%2)==1); }



bool IsWhiteSpace(char c) {
  return std::isspace(c);
}


int main () {
    std::string str2 = "Text\n with\tsome \t  whitespaces\n\n";
    std::cout << str2.size() << str2 << "\n";
    str2.erase(std::remove_if (str2.begin(), str2.end(), IsWhiteSpace), str2.end());
    std::cout << str2.size() << str2 << "\n";








  // int myints[] = {};            // 1 2 3 4 5 6 7 8 9
  std::vector<int> myints {2,3,4,4,5,6,7,8,9};

  // // bounds of range:
  // int* pbegin = myints;                          // ^
  // int* pend = myints+sizeof(myints)/sizeof(int); // ^                 ^

  /*std::vector<int> pend = */
  myints.erase(std::remove_if (myints.begin(), myints.end(), IsOdd), myints.end());   // 2 4 6 8 ? ? ? ? ?
                                                 // ^       ^
  std::cout << "the range contains:";
  // for (int* p=pbegin; p!=pend; ++p)
  //   std::cout << ' ' << *p;
  for (auto i: myints) {
    std::cout << i << " ";
  }


  std::cout << '\n';





  return 0;
}
