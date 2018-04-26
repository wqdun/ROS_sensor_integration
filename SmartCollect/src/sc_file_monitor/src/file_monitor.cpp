#include "file_monitor.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

FileMonitor::FileMonitor(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    LOG(INFO) << __FUNCTION__ << " start.";

    pubProjectArr_ = nh.advertise<sc_msgs::ProjectArr>("sc_project_array", 0);
}

FileMonitor::~FileMonitor() {
    LOG(INFO) << __FUNCTION__ << " start.";
    LOG(INFO) << "Goodbye.";
}


void FileMonitor::getProjects(const std::string &_projectPath, sc_msgs::ProjectArr::Ptr &_pProjectArr) {
    LOG(INFO) << __FUNCTION__ << " start, to monitor " << _projectPath;

    DIR *dir;
    dirent *ptr;

    if( !(dir = opendir(_projectPath.c_str())) ) {
        LOG(WARNING) << "Failed to open " << _projectPath;
        return;
    }

    while(ptr = readdir(dir)) {
        if( !(strcmp(ptr->d_name, ".")) ||
            !(strcmp(ptr->d_name, "..")) ) {
            LOG(INFO) << "Ignore: " << ptr->d_name;
            continue;
        }

        if(8 == ptr->d_type) {
            LOG(INFO) << "Ignore regular file: " << ptr->d_name;
            continue;
        }
        // directory
        if(4 == ptr->d_type) {
            const std::string project(ptr->d_name);
            LOG(INFO) << "I find a project: " << project;
            _pProjectArr->projects.push_back(project);
        }
    }

    closedir(dir);
    LOG(INFO) << "I find " << _pProjectArr->projects.size() << " projects.";
    return;
}

void FileMonitor::run() {
    LOG(INFO) << __FUNCTION__ << " start.";

    ros::Rate rate(0.2);
    while(ros::ok() ) {
        ros::spinOnce();
        rate.sleep();

        sc_msgs::ProjectArr::Ptr pProjectArr(new sc_msgs::ProjectArr() );

        (void)getProjects("/opt/smartc/record/", pProjectArr);
        pubProjectArr_.publish(pProjectArr);
    }

    LOG(INFO) << __FUNCTION__ << " end.";
}
