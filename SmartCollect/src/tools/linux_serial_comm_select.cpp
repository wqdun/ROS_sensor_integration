#include <stdio.h>
#include <string.h>
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

int name_arr[] = {
    38400, 19200, 9600, 4800, 2400, 1200, 300, 38400, 19200, 9600, 4800, 2400, 1200, 300
};

void set_speed(int fd, int speed) {
    int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for (int i = 0; i < sizeof(speed_arr) / sizeof(int); ++i) {
        if (speed == name_arr[i]) {
            tcflush (fd, TCIOFLUSH);
            cfsetispeed (&Opt, speed_arr[i]);
            cfsetospeed (&Opt, speed_arr[i]);
            status = tcsetattr (fd, TCSANOW, &Opt);
            if (status != 0) {
                perror("tcsetattr fd1");
                return;
              }
            tcflush(fd, TCIOFLUSH);
        }
    }
}

int set_Parity(int fd, int databits, int stopbits, int parity) {
    struct termios options;
    if(0 != tcgetattr(fd, &options) ) {
      perror("Setup Serial 1.");
      return (FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) {
    case 7:
      options.c_cflag |= CS7;
      break;
    case 8:
      options.c_cflag |= CS8;
      break;
    default:
      fprintf (stderr, "Unsupported data size\n");
      return (FALSE);
    }
    switch(parity) {
    case 'n':
    case 'N':
      options.c_cflag &= ~PARENB;   /* Clear parity enable */
      options.c_iflag &= ~INPCK;    /* Enable parity checking */
      break;
    case 'o':
    case 'O':
      options.c_cflag |= (PARODD | PARENB);
      options.c_iflag |= INPCK; /* Disnable parity checking */
      break;
    case 'e':
    case 'E':
      options.c_cflag |= PARENB;    /* Enable parity */
      options.c_cflag &= ~PARODD;
      options.c_iflag |= INPCK; /* Disnable parity checking */
      break;
    case 'S':
    case 's':           /*as no parity */
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      break;
    default:
        fprintf (stderr, "Unsupported parity\n");
        return (FALSE);
    }

    switch (stopbits) {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf (stderr, "Unsupported stop bits\n");
        return (FALSE);
    }
  /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush (fd, TCIFLUSH);
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;   /* Update the options and do it NOW */
    if (tcsetattr (fd, TCSANOW, &options) != 0) {
        perror ("SetupSerial 3");
        return (FALSE);
    }
    return (TRUE);
}

void signal_handler_IO (int status) {
    printf ("received SIGIO signale.\n");
    wait_flag = noflag;
}

void *testThread(void *arg) {
    cout << "I am a thread.\n";
}

int main() {
    pthread_t tId;
    int err;
    // if(0 != pthread_create(&tId, NULL, testThread, NULL) ) {
    //     cerr << "Error.\n";
    // }
    struct timeval timeout={0, 0};
    int fd = open("/dev/ttyUSB0", O_RDONLY | O_NONBLOCK);
    if(fd == -1) {
        perror("Serial port error\n");
        exit(1);
    }
    cout << "Open " << ttyname (fd) << " successfully\n";

    (void)set_speed(fd, 115200);
    if(FALSE == set_Parity (fd, 8, 1, 'N') ) {
        printf ("Set Parity Error\n");
        exit(0);
    }

    unsigned char buf[2000];
    fd_set rd;
    int nread = 0;

    while(1) {
        // FD_ZERO(&rd);
        // FD_SET(fd, &rd);
        // switch(select(fd + 1, &rd, NULL, NULL, NULL) ) {
        // case -1:
        //     cerr << "Select error.\n";
        //     exit(1);
        // case 0:
        //     // cout << "Fd not ready, select again.\n";
        //     break;
        // default:
        //     if(FD_ISSET(fd, &rd) ) {
                int t1 = 0;
                // while(1) {
                    nread = read(fd, buf, sizeof(buf) );
                    cout << dec << "nread: " << nread << "; t1: " << t1 << "\n";
                    if(nread <= 0)
                        // break;
                        continue;

                    // for(int k = 0; k < nread; ++k) {
                    // // printf("%d\n", buf[k]);
                    //     cout << hex << (int)buf[k]; // or nread
                    // }
                    // cout << "\n";
                    memset(buf, 0, sizeof(buf) );
                    ++t1;
                // }
        //     }
        // }
    }

    close(fd);
    return 0;
}

