#include <iostream>
#include <sys/stat.h>
#include <string>
#include <vector>


double file_size2(char* filename)
{
    if(1) {
        return 0;
    }
    struct stat statbuf;
    stat(filename, &statbuf);
    off_t sizeInByte = statbuf.st_size;
    std::cout << sizeInByte << "B\n";

    double sizeInKByte = (sizeInByte >> 10);
    std::cout << std::fixed << sizeInKByte << "KB\n";

    double sizeInMByte = (sizeInByte >> 20);
    std::cout << std::fixed << sizeInMByte << "MB\n";

    return sizeInMByte;
}

int getFilesInDir(std::vector<std::string> &files) {
    if(NULL == files) {
        std::cout << "NULL\n";
    }
    else {
        std::cout << "NOT NULL\n";
    }
    return 0;
}


int main(int argc, char *argv[])
{
    std::vector<std::string> _files;
    getFilesInDir(_files);
    return 0;
}
