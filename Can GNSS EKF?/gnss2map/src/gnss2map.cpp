// #include "gnss2map/gnss2map.hpp"

// Gnss_to_map::Gnss_to_map()
// : Node("gnss_to_map")
// {
//     map_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/eagleye2mgrs", 1);
//     fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
//         "/gnss", rclcpp::QoS{1}, std::bind(&Gnss_to_map::navsat_callback, this, std::placeholders::_1));

//     target_frame = this->declare_parameter<std::string>("target_frame", "map");
//     gnss_frame = this->declare_parameter<std::string>("gnss_frame", "gnss");
    
//     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
// }

// void Gnss_to_map::navsat_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr navsat_msg) {
//     geographic_msgs::msg::GeoPoint gps_msg;
//     gps_msg.latitude = navsat_msg->latitude;
//     gps_msg.longitude = navsat_msg->longitude;
//     gps_msg.altitude = navsat_msg->altitude;

//     geodesy::UTMPoint utm;
//     geodesy::fromMsg(gps_msg, utm);

//     gnss2map_msg.header = navsat_msg->header;
//     gnss2map_msg.header.frame_id = target_frame;

//     gnss2map_msg.pose.position.x = utm.easting;
//     gnss2map_msg.pose.position.y = utm.northing;
//     gnss2map_msg.pose.position.z = utm.altitude;

//     // Set orientation to default (identity quaternion)
//     gnss2map_msg.pose.orientation.x = 0.0;
//     gnss2map_msg.pose.orientation.y = 0.0;
//     gnss2map_msg.pose.orientation.z = 0.0;
//     gnss2map_msg.pose.orientation.w = 1.0;

//     map_pose_pub_->publish(gnss2map_msg);
// }

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<Gnss_to_map>());
//     rclcpp::shutdown();
//     return 0;
// }










// // #include "gnss2map/gnss2map.hpp"

// // Gnss_to_map::Gnss_to_map()
// // : Node("gnss_to_map")
// // {
// //     map_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gnss2mgrs", 1);
// //     fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
// //         "/gnss", rclcpp::QoS{1}, std::bind(&Gnss_to_map::navsat_callback, this, std::placeholders::_1));

// //     target_frame = this->declare_parameter<std::string>("target_frame", "map");
// //     gnss_frame = this->declare_parameter<std::string>("gnss_frame", "gnss");
    
// //     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
// //     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
// // }

// // void Gnss_to_map::navsat_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr navsat_msg) {
// //     geographic_msgs::msg::GeoPoint gps_msg;
// //     gps_msg.latitude = navsat_msg->latitude;
// //     gps_msg.longitude = navsat_msg->longitude;
// //     gps_msg.altitude = navsat_msg->altitude;

// //     geodesy::UTMPoint utm;
// //     geodesy::fromMsg(gps_msg, utm);

// //     gnss2map_msg.header = navsat_msg->header;
// //     gnss2map_msg.header.frame_id = target_frame;

// //     gnss2map_msg.pose.position.x = fmod(utm.easting, UTM2MGRS);
// //     gnss2map_msg.pose.position.y = fmod(utm.northing, UTM2MGRS);
// //     gnss2map_msg.pose.position.z = utm.altitude;

// //     // Set orientation to default (identity quaternion)
// //     gnss2map_msg.pose.orientation.x = 0.0;
// //     gnss2map_msg.pose.orientation.y = 0.0;
// //     gnss2map_msg.pose.orientation.z = 0.0;
// //     gnss2map_msg.pose.orientation.w = 1.0;

// //     map_pose_pub_->publish(gnss2map_msg);
// // }

// // int main(int argc, char * argv[])
// // {
// //     rclcpp::init(argc, argv);
// //     rclcpp::spin(std::make_shared<Gnss_to_map>());
// //     rclcpp::shutdown();
// //     return 0;
// // }












// // #include "gnss2map/gnss2map.hpp"

// // Gnss_to_map::Gnss_to_map()
// // : Node("gnss_to_map")
// // {
// //     map_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/eagleye2mgrs", 1);
    
// //     // Subscribe to the gnss_pose topic with PoseWithCovarianceStamped message type
// //     fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
// //         "/gnss", rclcpp::QoS{1}, std::bind(&Gnss_to_map::navsat_callback, this, std::placeholders::_1));

// //     target_frame = this->declare_parameter<std::string>("target_frame", "map");
// //     gnss_frame = this->declare_parameter<std::string>("gnss_frame", "gnss");
    
// //     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
// //     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
// // }

// // void Gnss_to_map::navsat_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr navsat_msg) {

// //     // Convert GPS coordinates to UTM
// //     geographic_msgs::msg::GeoPoint gps_msg;
// //     gps_msg.latitude = navsat_msg->latitude;
// //     gps_msg.longitude = navsat_msg->longitude;
// //     gps_msg.altitude = navsat_msg->altitude;

// //     geodesy::UTMPoint utm;
// //     geodesy::fromMsg(gps_msg, utm);

// //     // Create and populate the PoseStamped message
// //     geometry_msgs::msg::PoseWithCovarianceStamped gnss2map_msg;
// //     gnss2map_msg.header = pose_msg->header;
// //     gnss2map_msg.header.frame_id = target_frame;

// //     gnss2map_msg.pose= pose_msg->pose;

// //     gnss2map_msg.pose.pose.position.x = fmod(utm.easting, UTM2MGRS);
// //     gnss2map_msg.pose.pose.position.y = fmod(utm.northing, UTM2MGRS);
// //     gnss2map_msg.pose.pose.position.z = utm.altitude;

// //     // Publish the PoseStamped message
// //     map_pose_pub_->publish(gnss2map_msg);
// // }

// // int main(int argc, char * argv[])
// // {
// //     rclcpp::init(argc, argv);
// //     rclcpp::spin(std::make_shared<Gnss_to_map>());
// //     rclcpp::shutdown();
// //     return 0;
// // }





#include "gnss2map/gnss2map.hpp"

Gnss_to_map::Gnss_to_map()
: Node("gnss_to_map")
{
    map_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gnss2map", 1);
    
    // Subscribe to the gnss_pose topic with PoseWithCovarianceStamped message type
    fix_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/gnss_pose_with_covariance", rclcpp::QoS{1}, std::bind(&Gnss_to_map::pose_callback, this, std::placeholders::_1));

    target_frame = this->declare_parameter<std::string>("target_frame", "map");
    gnss_frame = this->declare_parameter<std::string>("gnss_frame", "gnss");
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void Gnss_to_map::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg) {
    // Extract position from the PoseWithCovarianceStamped message
    const auto& pose = pose_msg->pose.pose;
    const double latitude = pose.position.x; // Assuming position.x is latitude
    const double longitude = pose.position.y; // Assuming position.y is longitude
    const double altitude = pose.position.z; // Assuming position.z is altitude

    // Convert GPS coordinates to UTM
    geographic_msgs::msg::GeoPoint gps_msg;
    gps_msg.latitude = latitude;
    gps_msg.longitude = longitude;
    gps_msg.altitude = altitude;

    geodesy::UTMPoint utm;
    geodesy::fromMsg(gps_msg, utm);

    // Create and populate the PoseStamped message
    geometry_msgs::msg::PoseWithCovarianceStamped gnss2map_msg;
    gnss2map_msg.header = pose_msg->header;
    gnss2map_msg.header.frame_id = target_frame;

    gnss2map_msg.pose= pose_msg->pose;

    gnss2map_msg.pose.pose.position.x = fmod(utm.easting, UTM2MGRS);
    gnss2map_msg.pose.pose.position.y = fmod(utm.northing, UTM2MGRS);
    gnss2map_msg.pose.pose.position.z = utm.altitude;

    // Publish the PoseStamped message
    map_pose_pub_->publish(gnss2map_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gnss_to_map>());
    rclcpp::shutdown();
    return 0;
}
