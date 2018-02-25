#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdio.h>
// #include <direct.h>

using namespace std;



//定义全局变量var作为实参
char *var="name=shexinwei";

int
main(void)
{
    char *envVal;
    char *myenv;
    if ((envVal = getenv("PATH")) == NULL) {
        printf("not environment variable PATH\n");
    } else {
        printf("PATH=%s\n", envVal);
    }
    if (setenv("myenv", "li enhua", 0) == -1) {
        printf("setenv error\n");
     } else {
        myenv = getenv("myenv");
        printf("myenv=%s\n", myenv);
    }
    if (putenv("myenv1=haha") != 0) {
         printf("putenv error\n");
    } else {
         myenv = getenv("myenv1");
         printf("myenv1=%s\n", myenv);
    }
    exit(0);
}
