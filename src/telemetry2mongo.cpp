#include "telemetry2mongo.h"

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
    

Telemetry2mongo::Telemetry2mongo(const ros::NodeHandle& n,
        const double frequency, const std::string odom_topic, const std::string activation_service_name,
        const std::string uri_name, const std::string db_name, const std::string collection_name_telemetry,
        const std::string platform, const double threshold_time)
        : m_activationService()
        , m_odom_sub()
        , m_uri_name(uri_name)
        , m_db_name(db_name)
        , m_collection_name_telemetry(collection_name_telemetry)
        , m_platform(platform)
        , m_watchdog(ros::Time::now().toSec())
        , m_threshold_time(threshold_time)
        , m_frequency(frequency)
        , m_initialized(false)
        , m_activated(true)
        , m_valid_data(false)
{
    ros::NodeHandle nh;

    
    //services
    m_activationService = nh.advertiseService(activation_service_name, &Telemetry2mongo::activation_service, this);

    //subscribes
    m_odom_sub = nh.subscribe(odom_topic, 10, &Telemetry2mongo::updateOdom, this);
    
    //get time
    m_time = get_time();

    //mongo init
    mongocxx::instance instance{};

    
}
	
void Telemetry2mongo::run(double frequency) {
        
    ros::NodeHandle node;
    ros::Timer timer = node.createTimer(ros::Duration(1.0 / frequency), &Telemetry2mongo::iteration, this);
    ros::spin();

}

void Telemetry2mongo::iteration(const ros::TimerEvent& e) {
       
    ros::NodeHandle n;

    // Check initialized and the validity of the data
    if( m_initialized && 
        (ros::Time::now().toSec() - m_watchdog < m_threshold_time) ){
        m_valid_data = true;
    }else{
        if(m_initialized){
            ROS_WARN("Telemetry2mongo: invalid or missing data");
        }
        m_valid_data = false;
    }


    // Check if telemetry2mongo is active
    if(m_activated && m_valid_data){

        bool result = true;

        // Make a connection;
        mongocxx::uri uri(m_uri_name);
        mongocxx::client client(uri);

        // Access a database
        mongocxx::database db = client[m_db_name];

        // Access a collection
        mongocxx::collection coll = db[m_collection_name_telemetry];

        // Time
        m_time = get_time();

        // ID
        std::string telemetry_id = "tel_" + m_time;

        // Calculate orientation
        geometry_msgs::Point rpy = get_rpy(m_odom);

        // Check data
        bool check_position = (m_odom.pose.covariance[0]>100);
        bool check_orientation = std::isnan(rpy.x) ||
                std::isnan(rpy.y) ||
                std::isnan(rpy.z);

        // Create a document
	    auto builder = bsoncxx::builder::stream::document{};
	    auto doc_value_ = builder
            << "_id" << telemetry_id
            << "created" << m_time +".0";

        // X and Y
        if (check_position){
            doc_value_ = doc_value_
                << "x" << bsoncxx::types::b_null()
                << "y" << bsoncxx::types::b_null();
            ROS_WARN("Telemetry2mongo: position error");
        }else{
            doc_value_ = doc_value_
                << "x" << m_odom.pose.pose.position.x
                << "y" << m_odom.pose.pose.position.y;
        }

        // Theta
        if (check_orientation)
        {
            doc_value_ = doc_value_
                << "theta" << bsoncxx::types::b_null();
            ROS_WARN("Telemetry2mongo: orientation error");
        }else{
            doc_value_ = doc_value_
                << "theta" << rpy.z;
        }

        // Platform
        doc_value_ = doc_value_
            << "platform" << m_platform
            << "frequency" << m_frequency;

        // Close document  
        auto doc_value = doc_value_
            << bsoncxx::builder::stream::finalize;

	    bsoncxx::document::view view = doc_value.view();
	    bsoncxx::document::element element = view["_id"];
	    if(element.type() != bsoncxx::type::k_utf8) {
	      // Error
            ROS_WARN("Telemetry2mongo: element type wrong");
            result = false;
	    }
	    bsoncxx::stdx::optional<mongocxx::result::insert_one> result_ =
            coll.insert_one(view);

        ROS_INFO("Telemetry2mongo: %s uploaded on DB", telemetry_id.c_str());

    }
    

}



// Update Odometry
void Telemetry2mongo::updateOdom(const nav_msgs::Odometry::ConstPtr& msg) {
        
    m_odom = *msg;  

    if(!m_initialized){
        m_initialized=true;
    }
    
    m_watchdog = ros::Time::now().toSec();
}

// Activation service
bool Telemetry2mongo::activation_service(
    std_srvs::SetBool::Request &req,
    std_srvs::SetBool::Response &res) {

    try{
        m_activated=req.data;
        res.success = true;

        std::string message;

        message = "Telemetry2mongo: system -" + m_activated ? "true" : "false";

        res.message = message;
    }catch(...){
        res.success = false;
        res.message = "Telemetry2mongo: activation_service error";
    }

    return res.success;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "telemetry2mongo_node");
    ros::NodeHandle nh("~");
    
    double frequency;
    nh.param("frequency", frequency, 1.0);
    
    std::string odom_topic;
    nh.param<std::string>("odom_topic", odom_topic, "/ekf_localization_slam_node/slam_odom_magnetic");

    std::string activation_service_name;
    nh.param<std::string>("activation_service", activation_service_name, "/telemetry2mongo/active");

    std::string uri_name;
    nh.param<std::string>("uri_name", uri_name, "mongodb://localhost:27017");
    
    std::string db_name;
    nh.param<std::string>("db_name", db_name, "telemetry");

    std::string collection_name_telemetry;
    nh.param<std::string>("collection_name_telemetry", collection_name_telemetry, "telemetry");

    std::string platform;
    nh.param<std::string>("platform", platform, "UGV");

    double threshold_time;
    nh.param("threshold_time", threshold_time, 1.0);

    Telemetry2mongo telemetry2mongo(nh,
        frequency, odom_topic, activation_service_name,
        uri_name, db_name, collection_name_telemetry,
        platform, threshold_time);
    
    telemetry2mongo.run(frequency);

    return 0;
}
