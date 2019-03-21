#include <stdio.h>
#include <string.h>

#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include <iostream>
using namespace std;


#define FALSE -1
#define TRUE 0
#define flag 1
#define noflag 0

int wait_flag = noflag;
int STOP = 0;
int res;

int speed_arr[] = {
    B38400, B19200, B9600, B4800, B2400, B1200, B300, B38400, B19200, B9600, B4800, B2400, B1200, B300
};




void signal_handler_IO (int status) {
    printf ("received SIGIO signale.\n");
    wait_flag = noflag;
}

void *writeThread(void *arg) {
    FILE *pOutFile;
    if(!(pOutFile = fopen("out.710", "ab") ) ) {
        // cout << "Create file:" << mRecordFile << " failed, errno:" << errno;
        exit(1);
    }
    // cout << "Create file:" << mRecordFile << " successfully.";

    // (void)fwrite(&(scan->packets[0].data[0]), 1024, 1, pOutFile);

    fclose(pOutFile);


}

int main(int argc, char *argv[]) {
    cout << argv[1] << endl;
    // pthread_t tId;
    int err;
    FILE *pOutFile;
    if(!(pOutFile = fopen("5651_422.dat", "wb") ) ) {
        cout << "Create file: failed, errno:" << errno;
        exit(1);
    }
    cout << "Create file successfully: 5651_422.dat";
    // if(0 != pthread_create(&tId, NULL, writeThread, NULL) ) {
    //     cerr << "Error.\n";
    // }
    struct timeval timeout={0, 0};
    string term(argv[1]);
    term = "/dev/tty" + term;
    int fd = open(term.c_str(), O_RDONLY); // | O_NONBLOCK);
    if(fd == -1) {
        perror("Serial port error\n");
        exit(1);
    }
    cout << "Open " << ttyname(fd) << " successfully\n";

    (void)set_speed(fd, 115200);
    if(FALSE == set_Parity (fd, 8, 1, 'N') ) {
        printf ("Set Parity Error\n");
        exit(0);
    }

    unsigned char buf[50];
    fd_set rd;
    int nread = 0;
    string frameBuf("");

    while(1) {
        nread = read(fd, buf, sizeof(buf) );
        // string bufStr(buf);
        // fill_frame(bufStr, frameBuf);
        // now frameBuf should be human readable


        // parse
        // (void)fwrite(buf, nread, 1, pOutFile); // << C txt
        // if() set a member; then pub -> infor

        // cout << dec << "nread: " << nread << "\n";
        if(nread <= 0)
            // break;
            continue;

        for(int k = 0; k < nread; ++k) {
            printf("%02X ", buf[k]);
            // cout << hex << (int)buf[k]; // or nread
        }
        // cout << "\n";
        bzero(buf, sizeof(buf) );
    }

    fclose(pOutFile);
    close(fd);
    return 0;
}


// fill_frame(const string &_buf, string &_frameBuf) {
//     cout << __FUNCTION__ << " start.\n";
//     for(size_t i = 0; i < _buf.size(); ++i) {
//         switch(_buf[i]) {
//         case '$':



// }
