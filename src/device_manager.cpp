#include "ros/ros.h"
#include "smarthome/Register.h"
#include "smarthome/Sync.h"
#include "smarthome/Device.h"
#include "smarthome/Devices.h"

#include <vector>

// Info about registered devices
std::vector<smarthome::Device> registered_devices;
bool register_device(smarthome::Register::Request& req, smarthome::Register::Response& res) {
    ROS_INFO_STREAM("Registering new device: " << req.device.id);
    registered_devices.push_back(req.device);
    res.registered = true;
    return true;
}
bool sync(smarthome::Sync::Request& req, smarthome::Sync::Response& res) {
    ROS_INFO_STREAM("Syncing " << registered_devices.size() << " devices.");
    for(auto d = registered_devices.begin(); d != registered_devices.end(); d++) {
        res.devices.push_back(*d);
    }
    return true;
} 

int main(int argc, char* argv[]){
    ros::init(argc, argv, "device_manager");
    ros::NodeHandle nh;
    
    // Advertise device registration service
    ros::ServiceServer register_srv = nh.advertiseService("register_device", register_device);
    ros::ServiceServer sync_srv = nh.advertiseService("sync", sync);

    // Run
    ros::spin();

    return 0;
}