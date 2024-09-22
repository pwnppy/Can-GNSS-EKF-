// // #include "rclcpp/rclcpp.hpp"
// // #include "sensor_msgs/msg/nav_sat_fix.hpp"
// // #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// // #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// // class GNSSPoseEstimator : public rclcpp::Node {
// // public:
// //     GNSSPoseEstimator() : Node("gnss_pose_estimator") {
// //         gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
// //             "/gnss", 10, std::bind(&GNSSPoseEstimator::gnss_callback, this, std::placeholders::_1));

// //         pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
// //             "/raw_gnss_pose", 10);
// //     }

// // private:
// //     void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
// //         geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
// //         pose_msg.header.stamp = this->get_clock()->now();
// //         pose_msg.header.frame_id = "gnss";

// //         // Orientation 계산 (예: 지구의 중심 방향을 기준으로)
// //         tf2::Quaternion orientation;
// //         orientation.setRPY(0, 0, 0);  // 예: Roll, Pitch, Yaw
// //         pose_msg.pose.pose.orientation = tf2::toMsg(orientation);

// //         // 위치 설정
// //         pose_msg.pose.pose.position.x = msg->longitude;
// //         pose_msg.pose.pose.position.y = msg->latitude;
// //         pose_msg.pose.pose.position.z = 0;  // 예: 고도

// //         // Covariance 설정 (예: 단순한 대각행렬)
// //         pose_msg.pose.covariance[0] = 0.1;  // position.x variance
// //         pose_msg.pose.covariance[7] = 0.1;  // position.y variance
// //         pose_msg.pose.covariance[14] = 0.1; // position.z variance
// //         pose_msg.pose.covariance[21] = 0.1; // orientation variance

// //         pose_publisher_->publish(pose_msg);
// //     }

// //     rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
// //     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
// // };

// // int main(int argc, char **argv) {
// //     rclcpp::init(argc, argv);
// //     rclcpp::spin(std::make_shared<GNSSPoseEstimator>());
// //     rclcpp::shutdown();
// //     return 0;
// // }




// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/nav_sat_fix.hpp"
// #include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include <cmath>

// class GNSSPoseEstimator : public rclcpp::Node {
// public:
//     GNSSPoseEstimator() : Node("gnss_pose_estimator"), last_latitude_(0.0), last_longitude_(0.0), first_message_(true) {
//         gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
//             "/gnss", 10, std::bind(&GNSSPoseEstimator::gnss_callback, this, std::placeholders::_1));

//         pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
//             "/raw_gnss_pose", 10);
//     }

// private:
//     void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
//         geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
//         pose_msg.header.stamp = this->get_clock()->now();
//         pose_msg.header.frame_id = "gnss";

//         // Orientation 계산
//         if (!first_message_) {
//             double delta_lat = msg->latitude - last_latitude_;
//             double delta_lon = msg->longitude - last_longitude_;

//             double yaw = atan2(delta_lat, delta_lon);
//             tf2::Quaternion orientation;
//             orientation.setRPY(0, 0, yaw);  // Roll과 Pitch는 0으로 설정
//             pose_msg.pose.pose.orientation = tf2::toMsg(orientation);
//         } else {
//             tf2::Quaternion orientation;
//             orientation.setRPY(0, 0, 0);  // 첫 메시지의 경우 기본 방향 설정
//             pose_msg.pose.pose.orientation = tf2::toMsg(orientation);
//             first_message_ = false;
//         }

//         // 위치 설정
//         pose_msg.pose.pose.position.x = msg->longitude;
//         pose_msg.pose.pose.position.y = msg->latitude;
//         pose_msg.pose.pose.position.z = 0;  // 예: 고도

//         // Covariance 설정
//         pose_msg.pose.covariance[0] = 0.1;  // position.x variance
//         pose_msg.pose.covariance[7] = 0.1;  // position.y variance
//         pose_msg.pose.covariance[14] = 0.1; // position.z variance
//         pose_msg.pose.covariance[21] = 0.1; // orientation variance

//         pose_publisher_->publish(pose_msg);

//         // 마지막 위치 저장
//         last_latitude_ = msg->latitude;
//         last_longitude_ = msg->longitude;
//     }

//     rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
//     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    
//     double last_latitude_;
//     double last_longitude_;
//     bool first_message_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<GNSSPoseEstimator>());
//     rclcpp::shutdown();
//     return 0;
// }



#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>

class GNSSPoseEstimator : public rclcpp::Node {
public:
    GNSSPoseEstimator() : Node("gnss_pose_estimator"), last_latitude_(0.0), last_longitude_(0.0), first_message_(true) {
        gnss_subscriber_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gnss", 10, std::bind(&GNSSPoseEstimator::gnss_callback, this, std::placeholders::_1));

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/raw_gnss_pose", 10);
    }

private:
    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "gnss";

        // Orientation 계산
        double yaw = 0.0;
        if (!first_message_) {
            double delta_lat = msg->latitude - last_latitude_;
            double delta_lon = msg->longitude - last_longitude_;
            yaw = atan2(delta_lat, delta_lon);  // Yaw 계산

            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, yaw);  // Roll과 Pitch는 0으로 설정
            pose_msg.pose.pose.orientation = tf2::toMsg(orientation);

            // 쿼터니언에서 Roll, Pitch, Yaw 계산
            double roll, pitch, yaw;
            tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
            RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
        } else {
            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, 0);  // 첫 메시지의 경우 기본 방향 설정
            pose_msg.pose.pose.orientation = tf2::toMsg(orientation);
            first_message_ = false;
        }

        // 위치 설정
        pose_msg.pose.pose.position.x = msg->longitude;
        pose_msg.pose.pose.position.y = msg->latitude;
        pose_msg.pose.pose.position.z = 0;  // 예: 고도

        // Covariance 설정
        pose_msg.pose.covariance[0] = 0.1;  // position.x variance
        pose_msg.pose.covariance[7] = 0.1;  // position.y variance
        pose_msg.pose.covariance[14] = 0.1; // position.z variance
        pose_msg.pose.covariance[21] = 0.1; // orientation variance

        pose_publisher_->publish(pose_msg);

        // 마지막 위치 저장
        last_latitude_ = msg->latitude;
        last_longitude_ = msg->longitude;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    
    double last_latitude_;
    double last_longitude_;
    bool first_message_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNSSPoseEstimator>());
    rclcpp::shutdown();
    return 0;
}
