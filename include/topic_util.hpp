#ifndef TOPIC_UTIL_HPP
#define TOPIC_UTIL_HPP

#include <ros.h>
#include <master.h>
#include <vector>
#include <string>

namespace topic_util {
    std::vector<std::string>

    /*
        Filters topic list to include only topics under given namespace.

        @param topics list of topics to filter
        @param ns namespace to search under

        @return all topics under namespace ns
    */
    ros::master::V_TopicInfo filterTopics(ros::master::V_TopicInfo topics, std::string ns) {
        for(auto t = topics.begin(); t != topics.end(); t++) {

        }
    }
}

#endif