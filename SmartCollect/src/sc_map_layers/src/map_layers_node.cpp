#include "map_layers.h"
#define NDEBUG
// #undef NDEBUG
#include <glog/logging.h>

int main(int argc, char **argv) {
    // google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Got " << argc << " parameters.";
    if(2 != argc) {
        LOG(INFO) << "usage: exec projects.";
        return -1;
    }

    ros::init(argc, argv, "map_layers_node");
    const std::string projects(argv[1]);
    LOG(INFO) << "projects: " << projects;
    MapLayers mapLayerser(ros::NodeHandle(), ros::NodeHandle("~"), projects);
    mapLayerser.run();
    return 0;
}