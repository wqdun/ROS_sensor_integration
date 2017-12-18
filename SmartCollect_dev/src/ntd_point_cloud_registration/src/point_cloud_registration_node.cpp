#include "point_cloud_registration.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "point_cloud_registration");

    pointCloudRegistration pointCloudRegistrar;
    pointCloudRegistrar.run();
    LOG(INFO) << "Point Cloud Registration.";
    return 0;
}