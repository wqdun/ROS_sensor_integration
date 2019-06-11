#include <iostream>
#include <vector>

using namespace std;
int main(int argc, char const *argv[])
{
    vector<vector<int>> arr2d {
        {1, 2, 3},
        {3, 4},
        {5},
        {6}
    };

    int arr[][3] = {1, 2, 3, 4, 5, 6};

    int length = sizeof(arr) / sizeof(int);
    cout << sizeof(arr) / sizeof(int) << "\n";

    // for (int i = 0; i < arr2d.size(); ++i) {
    //     for (int j = 0; j < arr2d[i].size(); ++j) {
    //         cout << arr2d[i][j] << " ";
    //     }
    //     cout << "\n";
    // }


    for (int i = 0; i < length; ++i) {
        cout << *(*arr + i) << " ";
    }


    return 0;
}


