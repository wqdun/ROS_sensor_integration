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

using namespace std;

//Copyright (c) 2013 Uli Köhler
//License: Apache2.0

/**
 * A buffer-overflow-safe readlink() wrapper for C++.
 * @return A string containing the readlink()ed filename, or
 *         an empty string with errno being set to the appropriate error.
 *         See the readlink() man(2) for errno details.
 */

static std::string safeReadlink(const std::string& filename) {
    size_t bufferSize = 255;

    //Increase buffer size until the buffer is large enough
    while (1) {
        char* buffer = new char[bufferSize];
        size_t rc = readlink (filename.c_str(), buffer, bufferSize);
        if (rc == -1) {
            delete[] buffer;
            if(errno == EINVAL) {
                //We know that bufsize is positive, so
                // the file is not a symlink.
                errno = 0;
                return filename;
            } else if(errno == ENAMETOOLONG) {
                bufferSize += 255;
            } else {
                //errno still contains the error code
                return "";
            }
        } else {
            //Success! rc == number of valid chars in buffer
            errno = 0;
            string res(buffer, rc);
            cout << res << endl;
            return string(buffer, rc);
        }
    }
}
int
main ()
{
    char exec_name [BUFSIZ];

    readlink ("/proc/self/exe", exec_name, BUFSIZ);
    // sleep(10);

    puts (exec_name);

    cout << "RES:: " << safeReadlink("/proc/self/exe") << endl;

    std::string path(safeReadlink("/proc/self/exe") );
    cout << path.find("/devel/") << endl;
    cout << path.substr(0, path.find("devel/") ) << endl;
    char *p;
    if((p = getenv("USER")))
        printf("USER =%s\n",p);
    putenv("USER=AAAAAAAAA");
    printf("USER+%s\n",getenv("USER"));

    FILE *fpin;
    string cmd("cmd");
    if(NULL == (fpin = popen("./test.sh", "r") ) ) {
        cout << "Failed to " << cmd;
        exit(1);
    }
    int err = 0;
    if(0 != (err = pclose(fpin) ) ) {
        cout << "Failed to " << cmd << ", returns: " << err;
        exit(1);
    }

    return 0;
}

