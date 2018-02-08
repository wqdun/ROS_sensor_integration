#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>

using namespace std;



int main(void)
{
    string dir="./hello";
    if (access("hello", 0) == -1)
    {
        cout<<dir<<" is not existing"<<endl;
        cout<<"now make it"<<endl;
    }
}

