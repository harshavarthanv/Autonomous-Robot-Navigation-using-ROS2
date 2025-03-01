#pragma once

#include <functional>
#include <sstream>
#include <string>
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <typeinfo>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <nav2_msgs/action/follow_waypoints.hpp> 

struct PartInfo {
    geometry_msgs::msg::Point position; // Transformed position
    std::string type;
    std::string color;
};

struct WaypointParams {
    std::string type;
    std::string color;
};
namespace group16_final{
    class GoalPublisher :public rclcpp::Node {
    public:
        using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
        using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
        GoalPublisher(const std::string& node_name): Node(node_name),tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
        {
            declare_all_parameters();
            // Initialize the transform listener with the tf buffer
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            //initialize subscribers
            aruco_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
                "aruco_markers", 10,
                std::bind(&GoalPublisher::aruco_id, this, std::placeholders::_1));
            cam1_sub = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera1/image", rclcpp::SensorDataQoS(),
                std::bind(&GoalPublisher::cam1_frame, this, std::placeholders::_1));
            cam2_sub = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera2/image", rclcpp::SensorDataQoS(),
                std::bind(&GoalPublisher::cam2_frame, this, std::placeholders::_1));
            cam3_sub = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera3/image", rclcpp::SensorDataQoS(),
                std::bind(&GoalPublisher::cam3_frame, this, std::placeholders::_1));
            cam4_sub = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera4/image", rclcpp::SensorDataQoS(),
                std::bind(&GoalPublisher::cam4_frame, this, std::placeholders::_1));
            cam5_sub = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera5/image", rclcpp::SensorDataQoS(),
                std::bind(&GoalPublisher::cam5_frame, this, std::placeholders::_1));
            std::vector<geometry_msgs::msg::PoseArray> pose_array_vector;
            //initialize client
            client_ =
                rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");
            //initialize publisher
            odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&GoalPublisher::odom_callback, this, std::placeholders::_1));
            initial_pose_pub_ =
                this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "initialpose", 10);

            
            //sleep for 5 seconds
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

    private:
        /**
         * @brief Server response for goal
         */
        void goal_response_callback(
            std::shared_future<GoalHandleNavigation::SharedPtr> future);
        /**
         * @brief Server response feedback
         *
         * @param feedback
         */
        void feedback_callback(
            GoalHandleNavigation::SharedPtr,
            const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
        /**
         * @brief Server response result
         *
         * @param result
         */
        void result_callback(const GoalHandleNavigation::WrappedResult &result);
        /**
         * @brief Send goal to action server
         *
         */
        void send_goal();
        /**
         * @brief Odom subscriber callback to get the pose and initialize in rviz
         * 
         * @param msg 
         */
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const;
        /**
         * @brief Get order of waypoints by comparing marker_id
         * 
         */
        void read_params(); 
        /**
         * @brief Sends the marker_id get respective parameters
         * 
         */
        void declare_all_parameters();
        /**
         * @brief Declare the waypoint_params.yaml as parameters
         * 
         * @param aruco_key 
         */
        void process_waypoints(const std::string& aruco_key);
        /**
         * @brief Gets the aruco ID
         * 
         * @param msg 
         */
        void aruco_id(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> msg);
        /**
         * @brief Gets the pose from cam 1 with repective to world frame and logs it to the stucture
         * 
         * @param msg 
         */
        void cam1_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg);
        /**
         * @brief Gets the pose from cam 2 with repective to world frame and logs it to the stucture
         * 
         * @param msg 
         */
        void cam2_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg);
        /**
         * @brief Gets the pose from cam 3 with repective to world frame and logs it to the stucture
         * 
         * @param msg 
         */
        void cam3_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg);
        /**
         * @brief Gets the pose from cam 4 with repective to world frame and logs it to the stucture
         * 
         * @param msg 
         */
        void cam4_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg);
        /**
         * @brief Gets the pose from cam 5 with repective to world frame and logs it to the stucture
         * 
         * @param msg 
         */
        void cam5_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg);
        /**
         * @brief Finds the object color an type
         * 
         * @param part_color 
         * @param part_type 
         * @param bat_color 
         * @param bat_type 
         */
        void battery_data(int part_color, int part_type, std::string& bat_color, std::string& bat_type);
        /**
         * @brief Subscriber to "aruco_markers"
         * 
         */
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_sub;
        /**
         * @brief Subscriber to "/mage/camera1/image"
         * 
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam1_sub;
        /**
         * @brief Subscriber to "/mage/camera2/image"
         * 
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam2_sub;
        /**
         * @brief Subscriber to "/mage/camera3/image"
         * 
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam3_sub;
        /**
         * @brief Subscriber to "/mage/camera4/image"
         * 
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam4_sub;
        /**
         * @brief Subscriber to "/mage/camera5/image"
         * 
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr cam5_sub;
        /**
         * @brief Subscriber to "odom"
         * 
         */
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        /**
         * @brief Publisher to topic "/initialpose"
         *
         */
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
            initial_pose_pub_;
        /**
         * @brief Action client to server "navigate_to_pose"
         *
         */
        rclcpp_action::Client<FollowWaypoints>::SharedPtr client_;


        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::vector<PartInfo> cameraParts[5];
        std::vector<std::vector<geometry_msgs::msg::Point>> allPositions;
        int to_stop;
        int aruco_mark_id;
    };

    
}
