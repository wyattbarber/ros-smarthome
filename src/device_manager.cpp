#include <ros/ros.h>
#include "smarthome/Device.h"
#include "smarthome/Devices.h"

// Info about registered devices
bool register_device(Register::Request& req, Register::Response& res) {
    return true;
}

int main(int argc, int* argv[]){
    ros::NodeHandle nh();
    // Advertise device registration service
    nh.advertiseService('register_device', register_device);

}