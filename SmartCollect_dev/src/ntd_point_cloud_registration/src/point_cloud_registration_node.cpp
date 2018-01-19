#include "point_cloud_registration.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "point_cloud_registration");

    LOG(INFO) << "Point Cloud Registration.";
    pointCloudRegistration pointCloudRegistrar;
    pointCloudRegistrar.run();

    return 0;
}