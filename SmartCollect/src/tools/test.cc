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
// #include <direct.h>
#include <vector>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <errno.h>
#include <pwd.h>

using namespace std;

int
main() {
struct passwd *pwd = getpwuid(getuid());
printf("当前登陆的用户名为：%s\n", pwd->pw_name);
return 0;
}

// #include <stdio.h>
// #include <pwd.h>
// #include <unistd.h>
