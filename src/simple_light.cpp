#include "ros/ros.h"
#include "smarthome/Register.h"
#include "smarthome/Execute.h"
#include "smarthome/Device.h"
#include "smarthome/Query.h"
#include "std_msgs/Bool.h"

#include <string>

//#include "map_parser.hpp"

// Device status data
std::string dev_id;
bool on;

bool execute_handler(smarthome::Execute::Request& req, smarthome::Execute::Response& res){
    ROS_INFO_STREAM(dev_id << " --- EXECUTE request recieved.");

    if(req.command == "action.devices.commands.OnOff") {
        if(req.param_values[0] == "true") {
            ROS_INFO_STREAM(dev_id << " --- Turning light ON.");
            on = true;
        }
        else {
            ROS_INFO_STREAM(dev_id << "--- Turning light OFF.");
            on = false;
        }
    }
    else {
        ROS_ERROR_STREAM(dev_id << " --- Unregognized command: " << req.command);
        return false;
    }

    return true;
}

bool query_handler(smarthome::Query::Request& req, smarthome::Query::Response& res){
    ROS_INFO_STREAM(dev_id << " --- QUERY request recieved.");
    res.param_names = { "on" };
    res.param_values = { (on ? "true" : "false") };
    return true;
};

int main(int argc, char* argv[]) {
    // Initialize ROS node
    ros::init(argc, argv, "simple_light");
    ros::NodeHandle nh;
    std::string tmp_str = ros::this_node::getName();
    dev_id = tmp_str.erase(0,1);
    
    // Advertise execute and query services
    ros::ServiceServer exe_srv = nh.advertiseService(dev_id+"/execute", execute_handler);
    ros::ServiceServer que_srv = nh.advertiseService(dev_id+"/query", query_handler);

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