#include <unistd.h>

int main(void)
{
	char args[] = "-l";
	/*
	 * 执行/bin目录下的ls,第一参数为程序名ls,
	 * 第二个参数为-al,第三个参数为/etc/
	 */
	//execl("/bin/bash","sh ","cd",NULL,NULL);
	//execl("/bin/bash","sh ","cd catkin_ws",NULL,NULL);

	//execl("/bin/bash","sh ","./src/tools/interface.sh",NULL,NULL);
	//execl("/bin/bash","sh ","/~/catkin_ws/src/tools/interface.sh",NULL,NULL);
	execl("/bin/bash","bash ","/home/navi/catkin_ws/src/tools/interface.sh",NULL,NULL);
	return 0;
}
