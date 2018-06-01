#include "test.h"
Singleton* Singleton::st;
int main()
{

    Singleton* st = Singleton::getInstance();
    Singleton* st1 = Singleton::getInstance();

    if (st == st1)
    {
        cout << "两个对象是相同的实例。" << endl;
    }

    return 0;
}