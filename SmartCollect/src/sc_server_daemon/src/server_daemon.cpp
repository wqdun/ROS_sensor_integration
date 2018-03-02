#include "server_daemon.h"
// #define NDEBUG
#undef NDEBUG
#include <glog/logging.h>

ServerDaemon::ServerDaemon(ros::NodeHandle nh, ros::NodeHandle private_nh) {
    pub2Client_ = nh.advertise<sc_server_daemon::serverMsg>("sc_server2client", 0);

    subClient_ = nh.subscribe("sc_client_cmd", 10, &ServerDaemon::clientCB, this);
    serverMsg_.is_server_connected = false;
    projectName_.clear();
    serverMsg_.is_project_already_exist = -1;
    isClientConnected_ = false;
}

ServerDaemon::~ServerDaemon() {
}

void ServerDaemon::run() {
    ros::Rate rate(1);
    size_t freqDivider = 0;

    while(ros::ok() ) {
        ++freqDivider;
        freqDivider %= 256;
        ros::spinOnce();
        rate.sleep();

        // 0.25 Hz set is_project_already_exist -1: initial value
        if(0 == (freqDivider % 4) ) {
            if(!isClientConnected_) {
                serverMsg_.is_project_already_exist = -1;
            }
            isClientConnected_ = false;
        }

        serverMsg_.is_server_connected %= 64;
        ++serverMsg_.is_server_connected;
        DLOG(INFO) << __FUNCTION__ << " start, publish: " << (int64_t)(serverMsg_.is_server_connected);
        pub2Client_.publish(serverMsg_);
    }
}


void ServerDaemon::clientCB(const SmartCollector::clientCmd::ConstPtr& pClientMsg) {
    DLOG(INFO) << __FUNCTION__ << " start, client command: " << pClientMsg->project_name;
    isClientConnected_ = true;

    // shutdown
    if(1 == pClientMsg->system_cmd) {
        if(pClientMsg->password.empty() ) {
            LOG(WARNING) << "I can not power off with no password given.";
            return;
        }
        (void)runSystemCmd("sudo halt -p", pClientMsg->password);
        return;
    }
    // reboot
    if(2 == pClientMsg->system_cmd) {
        if(pClientMsg->password.empty() ) {
            LOG(WARNING) << "I can not reboot with no password given.";
            return;
        }
        (void)runSystemCmd("sudo reboot", pClientMsg->password);
        return;
    }

    // cleanup server processes
    if(3 == pClientMsg->system_cmd) {
        // add a "true" to avoid exit, and password is not necessary
        (void)runSystemCmd("pkill sc_integrate_; pkill roscameragps; killall nodelet; true", pClientMsg->password);
        return;
    }


    if(pClientMsg->project_name.empty() ) {
        LOG_FIRST_N(INFO, 1) << "Empty project name.";
        projectName_ = pClientMsg->project_name;
        serverMsg_.is_project_already_exist = -1;
        return;
    }

    // check if project is new
    if(pClientMsg->project_name == projectName_) {
        LOG_FIRST_N(INFO, 1) << "Same project: " << projectName_ << ", do nothing.";
        return;
    }
    projectName_ = pClientMsg->project_name;
    LOG(INFO) << "Got a new project: " << projectName_;

    std::string prjRecordDir("/opt/smartc/record/" + projectName_);
    if(0 == access(prjRecordDir.c_str(), 0) ) {
        LOG(WARNING) << prjRecordDir << " already exist.";
        serverMsg_.is_project_already_exist = 1;
        return;
    }
    serverMsg_.is_project_already_exist = 0;

    FILE *fpin;
    std::string launchScript("");

    if(isFileExist("/opt/smartc/src/tools/launch_project.sh") ) {
        launchScript = "/opt/smartc/src/tools/launch_project.sh";
    }
    else {
        LOG(ERROR) << "/opt/smartc/src/tools/launch_project.sh does not exist.";
        exit(1);
    }
    LOG(INFO) << "launchScript: " << launchScript;

    (void)runSystemCmd(
        "bash " + launchScript
        + " server "
        + pClientMsg->project_name
        , pClientMsg->password
    );

    return;
}

void ServerDaemon::runSystemCmd(const std::string &systemCmd, const std::string &passwd) {
    LOG(INFO) << __FUNCTION__ << " start, params: " << systemCmd << ", " << passwd;

    std::string cmd("");
    if(systemCmd.find("sudo") < systemCmd.length() ) {
        cmd = (
            "echo " + passwd + " | sudo -S su >/dev/null 2>&1; "
            + systemCmd
        );
    }
    else {
        cmd = systemCmd;
    }

    LOG(INFO) <<"Run " << cmd;

    FILE *fpin;
    if(NULL == (fpin = popen(cmd.c_str(), "r") ) ) {
        LOG(ERROR) << "Failed to " << cmd;
        exit(1);
    }
    int err = 0;
    if(0 != (err = pclose(fpin) ) ) {
        LOG(ERROR) << "Failed to " << cmd << ", returns: " << err;
        exit(1);
    }
    LOG(INFO) << "Run: " << cmd << " end.";
    return;
}

static bool isFileExist(const std::string& fileName) {
    std::fstream _file;
    _file.open(fileName.c_str(), std::ios::in);
    return (bool)(_file);
}

