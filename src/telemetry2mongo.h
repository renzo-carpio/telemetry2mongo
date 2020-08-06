#ifndef TELEMETRY2MONGO_H
#define TELEMETRY2MONGO_H

#include <ros/ros.h>
#include <sstream>
#include <cassert>
#include <vector>
#include <string>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <std_srvs/SetBool.h>

//mongo
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/instance.hpp>
#include <bsoncxx/document/value.hpp>
#include <bsoncxx/document/view.hpp>
#include <bsoncxx/document/element.hpp>
#include <bsoncxx/document/view_or_value.hpp>
#include <bsoncxx/types.hpp>

//filesystem
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <sys/stat.h>


#include "date.h"

#define PI 3.14159265359

class Telemetry2mongo
{

public:
       
    Telemetry2mongo(const ros::NodeHandle& n,
        const double frequency, const std::string odom_topic, const std::string activation_service_name,
        const std::string uri_name, const std::string db_name, const std::string collection_name_telemetry,
        const std::string platform, const double threshold_time);


    void run(double frequency);

private:

    void iteration(const ros::TimerEvent& e);

    // Update Odometry
    void updateOdom(const nav_msgs::Odometry::ConstPtr& msg);

    // Activation service
    bool activation_service(
        std_srvs::SetBool::Request &req,
        std_srvs::SetBool::Response &res);

    
private:

    ros::ServiceServer m_activationService;
    nav_msgs::Odometry m_odom;
    ros::Subscriber m_odom_sub;
    std::string m_time;
    std::string m_uri_name;
    std::string m_db_name;
    std::string m_collection_name_telemetry;
    std::string m_platform;
    double m_frequency;
    bool m_initialized;
    bool m_activated;
    bool m_valid_data;

    // Watchdog
    double m_watchdog;
    double m_threshold_time;

};

// This function returns the current time in the following format
//   YYYY-MM-DD_hh-mm-ss 
std::string get_time();

// This function returns a structure with the Euler angles rpy as a Point rosmsg 
geometry_msgs::Point get_rpy(nav_msgs::Odometry odom);


#endif        
