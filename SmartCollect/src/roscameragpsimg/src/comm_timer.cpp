#include "comm_timer.h"
#include <fstream>
#include "lock.h"

#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>


using namespace std;

vector<string> parsed_data;
string global_gps = " ";
string imu_string;
int    gps_imu_state = 0;
//path control
extern string pathSave_str;
extern string imu_path;
//save control
extern int is_save_cam;

//CMutex mymutex_time;
extern CMutex mymutex;
static double string2double(const string& str)
{
    std::stringstream iss(str);
    double num;
    iss >> num;
    return num;
}

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if(tcgetattr(fd, &oldtio) != 0)
    {
        perror("Setup Serial 1.");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= (CLOCAL | CREAD);
    newtio.c_cflag &= ~CSIZE;

    switch(nBits)
   {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch(nEvent)
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch(nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }

    if(1 == nStop)
    {
        newtio.c_cflag &= ~CSTOPB;
    }
    else
    if(2 == nStop)
    {
        newtio.c_cflag |= CSTOPB;
    }
    else
    {
        perror("Setup nStop unavailable.");
        return -1;
    }

    tcflush(fd, TCIFLUSH);

    newtio.c_cc[VTIME] = 100; // time out 15s重要
    newtio.c_cc[VMIN] = 0; // Update the option and do it now 返回的最小值  重要

    if(0 != tcsetattr(fd, TCSANOW, &newtio))
    {
        perror("Com setup error.");
        return -1;
    }

    // printf("set done!\n\r");
    return 0;
}

int get_time(/*ros::NodeHandle* nh*/)
{
    char buf[1024];

    int fd1 = open("/dev/ttyS0", O_RDONLY);// | O_NONBLOCK); // 打开串口 // fd1 = open("/dev/ttyUSB0", O_RDWR);
    if(-1 == fd1)
    {
        printf("fd1 == -1\n");
        exit(1);
    }
    else
    {
        LOG(INFO)<<"comm open success!";
    }

    ros::NodeHandle nh_time;
    ros::Publisher    pub_5651  = nh_time.advertise<roscameragpsimg::imu5651>("imu_string", 1000);

    roscameragpsimg::imu5651  msg;

    // setup port properties
    int nset1 = set_opt(fd1, 115200, 8, 'N', 1); // 设置串口属性
    if(-1 == nset1)
    {
        exit(1);
    }

    std::string frameBuf;
    long freq = 0;
    timespec t1, t2;
    long last_time_ns;
    long time_when_get_frame_s, time_when_get_frame_ns;
    double time_s = 0;

    int last_minute = 0;
    double currTime = 0;
    string last_time_gps = " ";
    string pre_time_gps  = " ";
    double time_s_start, time_s_end;
    int    counts_t = 0;
    string GPS_week_time_str_cur=" ";
    string temp_path = " ";
    public_tools::PublicTools::generateFileName(imu_path, temp_path);

    string imupath_str = imu_path +temp_path + "_rt_track.txt";

    fstream file;
    while(1)
    {

        memset(buf, 0, 1024);
        int nread = read(fd1, buf, 1024); // 读串口

        if(nread <= 0)
        {
            continue;
        }
        bool is_frame_completed = false;
        for(size_t i = 0; i < nread; ++i)
        {
            is_frame_completed = false;
            string frame_complete = "";
            switch(buf[i])
            {
            case '$':
                clock_gettime(CLOCK_REALTIME, &t2);
                // cout << "time_end  :" << t2.tv_nsec << endl;
                time_when_get_frame_s = t2.tv_sec;
                time_when_get_frame_ns = t2.tv_nsec;
                time_s = (double)time_when_get_frame_s + (double)time_when_get_frame_ns / 1000000000.0;

                frameBuf = buf[i];
                break;
            case '\r':
                break;
            case '\n':
                time_when_get_frame_s %= 3600;

                is_frame_completed = true;
                frame_complete = frameBuf;
                frameBuf.clear();
                break;

            default:
                frameBuf += buf[i];
            }

            if(is_frame_completed)
            {


                int ret = mymutex.Trylock();
                DLOG(INFO) << "get_time lock result: " << ret;
                if(0 != ret)
                {
                    LOG(WARNING) << "Failed to tryLock, return result: " << ret;
                    continue;
                }

                boost::split(parsed_data, frame_complete, boost::is_any_of( ",*" ), boost::token_compress_on);
                // e.g. "279267.900"
                GPS_week_time_str_cur = parsed_data[2];
                double GPS_week_time = string2double(GPS_week_time_str_cur);
                global_gps = GPS_week_time_str_cur;

                if(parsed_data.size()>=17)
                {
                    msg.GPSWeek = parsed_data[1];
                    msg.GPSTime = parsed_data[2];
                    msg.Heading = parsed_data[3];
                    msg.Pitch   = parsed_data[4];
                    msg.Roll    = parsed_data[5];
                    msg.Latitude  = parsed_data[6];
                    msg.Longitude = parsed_data[7];
                    msg.Altitude  = parsed_data[8];
                    msg.Vel_east  = parsed_data[9];
                    msg.Vel_north = parsed_data[10];
                    msg.Vel_up    = parsed_data[11];
                    msg.Baseline = parsed_data[12];
                    msg.NSV1_num = parsed_data[13];
                    msg.NSV2_num = parsed_data[14];
                    msg.Status   = parsed_data[15];
                    pub_5651.publish(msg);
                }
                mymutex.Unlock();

                if(msg.Latitude.find("0.0000") < msg.Latitude.size() ) {
                    LOG_EVERY_N(INFO, 1000) << "msg.Latitude is invalid: " << msg.Latitude;
                    continue;
                }
                const double sysWeekSec = ros::Time::now().toSec() - 24 * 3600 * 3;
                const int timeErr = (int)sysWeekSec % (24 * 7 * 3600) - GPS_week_time;
                LOG_EVERY_N(INFO, 1000) << "Unix time and GPS week time diff: " << timeErr;
                LOG_FIRST_N(INFO, 1) << "Invalid GPS frame when time diff > 20 min.";
                if(timeErr < -1200 || timeErr > 1200) {
                    continue;
                }
                file.open(imupath_str, ios::out|ios::app);
                if(!file)
                {
                    LOG(ERROR) << "Failed to open " << imupath_str;
                    continue;
                }
                file << frame_complete << endl;
                file.close();
            }

        }
        //GPS精确时间
        //获取已有时间
        //获取先前时间
        //时间相同
        //第一次获取系统时间
        //其他获取系统时间现有时间
        //时间不同
        //更新现有时间和已有时间
        long   time_when_get_frame_ss, time_when_get_frame_nss;
        long   time_when_get_frame_se, time_when_get_frame_nse;
        timespec time_sys_end, time_sys_start;
        double time_difference;

        if(pre_time_gps != GPS_week_time_str_cur)
        {
                pre_time_gps = GPS_week_time_str_cur;
                counts_t = 0;
        }
        if(pre_time_gps == GPS_week_time_str_cur && counts_t == 0)
        {
            clock_gettime(CLOCK_REALTIME, &time_sys_start);
            time_when_get_frame_ss = time_sys_start.tv_sec;
            time_when_get_frame_nss = time_sys_start.tv_nsec;
            time_s_start = (double)time_when_get_frame_ss + (double)time_when_get_frame_nss / 1000000000.0;
            counts_t = counts_t + 1;

        }
        if(pre_time_gps == GPS_week_time_str_cur && counts_t > 0)
        {
            clock_gettime(CLOCK_REALTIME, &time_sys_end);
            time_when_get_frame_se = time_sys_end.tv_sec;
            time_when_get_frame_nse = time_sys_end.tv_nsec;
            time_s_end = (double)time_when_get_frame_se + (double)time_when_get_frame_nse / 1000000000.0;
            counts_t = counts_t + 1;
            time_difference = time_s_end - time_s_start;

            double GPS_week_times = string2double(GPS_week_time_str_cur);
            GPS_week_times = GPS_week_times + time_difference;

            int ret = mymutex.Trylock();
            if(ret != 0)
            {
               LOG(WARNING) << "Faild to trylock, return ret:" << ret;
               continue;
            }

            if(parsed_data.size() >= 17&& parsed_data[2].size()>2)
            {
                global_gps = std::to_string(GPS_week_times);
            }
            mymutex.Unlock();
        }
    }

    close(fd1);
    return 0;
}


