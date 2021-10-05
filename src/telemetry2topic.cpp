#include "telemetry2topic.h"

// Get time
/*std::string get_time() {
    std::time_t t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    
    std::ostringstream oss;
    //oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    
    auto str = oss.str();

    return str;
}*/

std::string get_time()
{
    using namespace std::chrono;
    auto now = time_point_cast<milliseconds>(system_clock::now());
    return date::format("%Y-%m-%d_%H-%M-%S", now);
}

    
// Get roll, pitch and yaw
geometry_msgs::Point get_rpy(nav_msgs::Odometry odom) {
    tf::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    geometry_msgs::Point rpy;
    
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    
    return rpy;
    }
    

Telemetry2topic::Telemetry2topic(const ros::NodeHandle& n,
        const double frequency, const std::string odom_topic, const std::string gps_topic,
        const std::string mission_topic, const std::string payload_topic, const std::string activation_service_name,
        const std::string platform, const double threshold_time)
        : m_activationService()
        , m_odom_sub()
        , m_gps_sub()
        , m_mission_sub()
        , m_payload_pub()
        , m_platform(platform)
        , m_watchdog1(ros::Time::now().toSec())
        , m_watchdog2(ros::Time::now().toSec())
        , m_watchdog3(ros::Time::now().toSec())
        , m_threshold_time(threshold_time)
        , m_frequency(frequency)
        , m_initialized1(false)
        , m_initialized2(false)
        , m_initialized3(false)
        , m_activated(true)
        , m_valid_data(false)
{
    ros::NodeHandle nh;

    
    //services
    m_activationService = nh.advertiseService(activation_service_name, &Telemetry2topic::activation_service, this);

    //subscribes
    m_odom_sub = nh.subscribe(odom_topic, 10, &Telemetry2topic::updateOdom, this);
    m_gps_sub = nh.subscribe(gps_topic, 10, &Telemetry2topic::updateGps, this);
    m_mission_sub = nh.subscribe(mission_topic, 10, &Telemetry2topic::updateMission, this);

    //publisher
    m_payload_pub = nh.advertise<std_msgs::String>(payload_topic, 1);
    
    //get time
    m_time = get_time();

    //mongo init
    mongocxx::instance instance{};

    
}
	
void Telemetry2topic::run(double frequency) {
        
    ros::NodeHandle node;
    ros::Timer timer = node.createTimer(ros::Duration(1.0 / frequency), &Telemetry2topic::iteration, this);
    ros::spin();

}

void Telemetry2topic::iteration(const ros::TimerEvent& e) {
       
    ros::NodeHandle n;

    // Check initialized and the validity of the data
    if( m_initialized1 && m_initialized2 && m_initialized3 &&
        (ros::Time::now().toSec() - m_watchdog1 < m_threshold_time) &&
        (ros::Time::now().toSec() - m_watchdog2 < m_threshold_time) &&
        (ros::Time::now().toSec() - m_watchdog3 < m_threshold_time) ){
        m_valid_data = true;
    }else{
        if(m_initialized1 && m_initialized2 && m_initialized3){
            ROS_WARN("Telemetry2topic: invalid or missing data");
        }
        m_valid_data = false;
    }


    // Check if telemetry2topic is active
    if(m_activated && m_valid_data){

        bool result = true;

        // Time
        m_time = get_time();

        // ID
        std::string telemetry_id = "tel_" + m_time;

        // Calculate orientation
        geometry_msgs::Point rpy = get_rpy(m_odom);

        // Create a document
	    auto builder = bsoncxx::builder::stream::document{};
	    auto doc_value_ = builder
            << "id" << telemetry_id
            << "created" << m_time // +".0"
            << "id_mission_part" << m_mission.data
            << "status" << "in_progress" //creare callback per inviare "executed"
            << "latitude" << m_gps.latitude
            << "longitude" << m_gps.longitude
            << "altitude" << m_gps.altitude
            << "yaw" << rpy.z
            << "pitch" << rpy.y
            << "roll" << rpy.x
            << "platform" << m_platform;

        // Close document  
        auto doc_value = doc_value_
            << bsoncxx::builder::stream::finalize;

	    bsoncxx::document::view view = doc_value.view();
	    bsoncxx::document::element element = view["id"];
	    if(element.type() != bsoncxx::type::k_utf8) {
	      // Error
            ROS_WARN("Telemetry2topic: element type wrong");
            result = false;
	    }
        std::string rawResult = bsoncxx::to_json(doc_value.view());

        ROS_INFO("Telemetry2topic: %s", rawResult.c_str());
        m_payload.data= rawResult.c_str();
        m_payload_pub.publish(m_payload);
    }
    

}


// Update Odometry
void Telemetry2topic::updateOdom(const nav_msgs::Odometry::ConstPtr& msg) {
        
    m_odom = *msg;  

    if(!m_initialized1){
        m_initialized1=true;
    }
    
    m_watchdog1 = ros::Time::now().toSec();
}

// Update GPS
void Telemetry2topic::updateGps(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        
    m_gps = *msg;  

    if(!m_initialized2){
        m_initialized2=true;
    }
    
    m_watchdog2 = ros::Time::now().toSec();
}

// Update Mission
void Telemetry2topic::updateMission(const std_msgs::String::ConstPtr& msg) {
        
    m_mission = *msg; 

    if(!m_initialized3){
        m_initialized3=true;
    }
    
    m_watchdog3 = ros::Time::now().toSec();
}

// Activation service
bool Telemetry2topic::activation_service(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res) {

    try{
        m_activated=req.data;
        res.success = true;

        std::string message;

        message = "Telemetry2topic: system -" + m_activated ? "true" : "false";

        res.message = message;
    }catch(...){
        res.success = false;
        res.message = "Telemetry2topic: activation_service error";
    }

    return res.success;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "telemetry2topic_node");
    ros::NodeHandle nh("~");
    
    double frequency;
    nh.param("frequency", frequency, 1.0);
    
    std::string odom_topic;
    nh.param<std::string>("odom_topic", odom_topic, "/ekf_slam_node/slam_odom");

    std::string gps_topic;
    nh.param<std::string>("gps_topic", gps_topic, "/ekf_slam_node/slam_fix");

    std::string mission_topic;
    nh.param<std::string>("mission_topic", mission_topic, "/mission");

    std::string activation_service_name;
    nh.param<std::string>("activation_service", activation_service_name, "/telemetry2topic/active");

    std::string payload_topic;
    nh.param<std::string>("payload_topic", payload_topic, "/payload");

    std::string platform;
    nh.param<std::string>("platform", platform, "UGV");

    double threshold_time;
    nh.param("threshold_time", threshold_time, 1.5);


    Telemetry2topic telemetry2topic(nh,
        frequency, odom_topic, gps_topic,
        mission_topic, payload_topic, activation_service_name,
        platform, threshold_time);
    
    telemetry2topic.run(frequency);

    return 0;
}
