#ifndef __BASE_SERVER_H__
#define __BASE_SERVER_H__

#include <string>
#include "sc_msgs/MonitorMsg.h"

class BaseServer {
public:
    virtual ~BaseServer();
    void UpdateProjectInfo(const std::string &projectInfo);
    virtual void Run() = 0;


protected:
    sc_msgs::MonitorMsg _monitorMsg_;


// private:
};

#endif
