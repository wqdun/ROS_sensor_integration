#ifndef __SERVER_FACTORY_H__
#define __SERVER_FACTORY_H__

#include <string>

class BaseServer;
class ServerFactory {
public:
    void SetSensorType(const std::string &sensorTypeYaml);
    BaseServer *CreateServer();


private:
    std::string __imuType_;
};

#endif
