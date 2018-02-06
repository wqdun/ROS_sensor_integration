#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

int main( void )
{
    FILE   *stream;
    FILE    *wstream;
    char   buf[1024];

    memset( buf, '/0', sizeof(buf) );//初始化buf,以免后面写如乱码到文件中
    stream = popen( "ls KKKK; sleep 2", "r" ); //将“ls －l”命令的输出 通过管道读取（“r”参数）到FILE* stream
    // popen( "sleep 10", "r" );
    wstream = fopen( "test_popen.txt", "w+"); //新建一个可写的文件

    fread( buf, sizeof(char), sizeof(buf),  stream);  //将刚刚FILE* stream的数据流读取到buf中
    fwrite( buf, 1, sizeof(buf), wstream );//将buf中的数据写到FILE    *wstream对应的流中，也是写到文件中

    // // pclose( stream );
    fclose( wstream );

    // system("sleep 10");

    cout << "Helo " << endl;


    putenv("TEST_NAME3=a");
    printf("TEST_NAME3=%s/n", getenv("TEST_NAME3"));

    putenv("TEST_NAME3=b");
    printf("TEST_NAME3=%s/n", getenv("TEST_NAME3"));

    setenv("TEST_NAME3", "c", 0);
    printf("TEST_NAME3=%s/n", getenv("TEST_NAME3"));

    setenv("TEST_NAME3", "d", 2);
    printf("TEST_NAME3=%s/n", getenv("TEST_NAME3"));

    printf("%d/n", unsetenv("TEST_NAME3"));
    printf("%d/n", unsetenv("TEST_NAME3"));

    return 0;
}
