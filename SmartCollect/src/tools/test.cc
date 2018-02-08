#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <QtCore/QString>

using namespace std;


// g++ test.cc -I/usr/include/qt4/ -lQtCore







int main( void )
{
    double a = 63.673456789;

    QString s = QString::number(a, 'f', 6);

    cout << s.toStdString() << endl;
    return 0;
}
