#include "ros/ros.h"
#include "smarthome/Register.h"
#include "smarthome/Device.h"
#include "std_msgs/Bool.h"

#include <string>

// Device status data
std::string dev_id;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "simple_light");
    ros::NodeHandle nh;
    std::string tmp_str = ros::this_node::getName();
    dev_id = tmp_str.erase(0,1);

    // Register with device_manager node
    ros::ServiceClient register_srv = nh.serviceClient<smarthome::Register>("register_device");
    smarthome::Register srv;
    srv.request.device.id =             dev_id;
    srv.request.device.type =           "action.devices.types.LIGHT";
    srv.request.device.traits =         {"action.devices.traits.OnOff"};
    srv.request.device.name =           dev_id;
    srv.request.device.nicknames =      {"light", "simple_light"};
    srv.request.device.default_names = {"light"};
    srv.request.device.will_report_state = false;
    srv.request.device.room_hint = "Living Room";
    srv.request.device.manufacturer = "Wyatt Corp.";
    srv.request.device.model = "Development";
    srv.request.device.sw_version = "0.0.1";
    srv.request.device.hw_version = "0.0.0";
    srv.request.device.device_id = dev_id;
    while(true){
        if(register_srv.call(srv)){
            ROS_INFO("Regisered with manager.");
            break;
        }
        else {
            ROS_ERROR("Failed to register with manager. Trying again in 1 second.");
            ros::Duration(1.0).sleep();
        }
    }

    // Run
    ros::spin();
    
    return 0;
}