#include "GoalPublisher.hpp"



// Member function implementations
namespace group16_final{

    // sends the captured marker_id to get respective parameter
    void GoalPublisher::read_params() {
            if (to_stop == 6){
                std::string marker_action = "aruco_" + std::to_string(aruco_mark_id);
                process_waypoints(marker_action);
                
            }
    }

    // Declares the waypoint_params.yaml as parameters
    void GoalPublisher::declare_all_parameters() {
        std::string aruco_detected = "aruco_" + std::to_string(aruco_mark_id);
            
            for (int i = 1; i <= 5; ++i) {
                std::string base_param = aruco_detected + std::string(".wp") + std::to_string(i);
                this->declare_parameter(base_param + ".type", rclcpp::ParameterValue(std::string()));
                this->declare_parameter(base_param + ".color", rclcpp::ParameterValue(std::string()));
            }
    }
    // identify parameter and initiates send_goal in required order of wp
    void GoalPublisher::process_waypoints(const std::string& aruco_key) {
        std::map<std::string, WaypointParams> waypoints;
            for (int i = 1; i <= 5; ++i) {
                std::string base_param = aruco_key + ".wp" + std::to_string(i);
                WaypointParams wp;
                this->get_parameter(base_param + ".type", wp.type);
                this->get_parameter(base_param + ".color", wp.color);
                std::transform(wp.type.begin(), wp.type.end(), wp.type.begin(), ::toupper);
                std::transform(wp.color.begin(), wp.color.end(), wp.color.begin(), ::toupper);

                waypoints["wp" + std::to_string(i)] = wp;
            }
            for (const auto& wp : waypoints) {
                RCLCPP_INFO(this->get_logger(), "Waypoint %s: Type = %s, Color = %s", 
                            wp.first.c_str(), wp.second.type.c_str(), wp.second.color.c_str());
            }
            
            for (const auto& pair : waypoints) {
                const WaypointParams& wp = pair.second;
                std::vector<geometry_msgs::msg::Point> matchedPositions;
                
                for (const auto& cameraPart : cameraParts) {
                    for (const auto& part : cameraPart) {
                        if (wp.type == part.type && wp.color == part.color) {
                            matchedPositions.push_back(part.position);
                        }
                    }
                }
                allPositions.push_back(matchedPositions);
            }
            for (const auto& positions : allPositions) {
                for (const auto& pos : positions) {
                    RCLCPP_INFO(this->get_logger(), " Ordered Positions : x = %f, y = %f, z = %f", pos.x, pos.y, pos.z);
                }
            }
            send_goal();
    }

    //gets the aruco_id 
    void GoalPublisher::aruco_id(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> msg) {
        for (auto aruco_mark_id : msg->marker_ids)
            {
                RCLCPP_INFO(this->get_logger(), "Detected ArUco Marker ID: %ld", aruco_mark_id);
                
            }
            to_stop = to_stop+1;
            RCLCPP_INFO(this->get_logger(), "aruco subscriber stopped");
            aruco_sub.reset();
            if(to_stop==6){
                read_params();
            }
    }
    //Gets the pose with respect to world frame and logs it for 5 camera
    
    void GoalPublisher::cam1_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg) {
        cameraParts[0].clear(); 
            for (const auto &part_pose : msg->part_poses)
            {
                geometry_msgs::msg::PoseStamped stamped_in_pose;
                stamped_in_pose.header.frame_id = "camera1_frame";
                stamped_in_pose.header.stamp = this->get_clock()->now();
                stamped_in_pose.pose = part_pose.pose;
                geometry_msgs::msg::PoseStamped stamped_out_pose;
                stamped_out_pose = tf_buffer_->transform(stamped_in_pose, "map");
                int part_color = part_pose.part.color;
                int part_type = part_pose.part.type;
                std::string bat_color, bat_type;
                battery_data(part_color, part_type, bat_color, bat_type);
                RCLCPP_INFO(this->get_logger(), "Camera1 clor type detected: Color: %s, Type: %s", bat_color.c_str(), bat_type.c_str());
                PartInfo info;
                info.position = stamped_out_pose.pose.position;
                info.type = bat_type;  
                info.color = bat_color; 
                cameraParts[0].push_back(info);
                RCLCPP_INFO(this->get_logger(), "Camera1 pose: x = %f, y = %f, z = %f",info.position.x, info.position.y, info.position.z);
            }
            to_stop = to_stop+1;
            RCLCPP_INFO(this->get_logger(), "cam1 subscriber stopped");
            cam1_sub.reset();
            if(to_stop==6){
                read_params();
            }
    }
    //Camera 2
    void GoalPublisher::cam2_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg) {
        cameraParts[1].clear();
            for (const auto &part_pose : msg->part_poses)
            {
                geometry_msgs::msg::PoseStamped stamped_in_pose;
                stamped_in_pose.header.frame_id = "camera2_frame";
                stamped_in_pose.header.stamp = this->get_clock()->now();
                stamped_in_pose.pose = part_pose.pose;
                geometry_msgs::msg::PoseStamped stamped_out_pose;
                stamped_out_pose = tf_buffer_->transform(stamped_in_pose, "map");
                int part_color = part_pose.part.color;
                int part_type = part_pose.part.type;
                std::string bat_color, bat_type;
                battery_data(part_color, part_type, bat_color, bat_type);
                RCLCPP_INFO(this->get_logger(), "Camera2 clor type detected: Color: %s, Type: %s", bat_color.c_str(), bat_type.c_str());
                PartInfo info;
                info.position = stamped_out_pose.pose.position;
                info.type = bat_type;  
                info.color = bat_color;
                cameraParts[1].push_back(info);
                RCLCPP_INFO(this->get_logger(), "Camera2 pose: x = %f, y = %f, z = %f",info.position.x, info.position.y, info.position.z);
            }
            to_stop = to_stop+1;
            RCLCPP_INFO(this->get_logger(), "cam2 subscriber stopped");
            cam2_sub.reset();
            if(to_stop==6){
                read_params();
            }
    }

    //Camera 3
    void GoalPublisher::cam3_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg) {
        cameraParts[2].clear();
            for (const auto &part_pose : msg->part_poses)
            {
                geometry_msgs::msg::PoseStamped stamped_in_pose;
                stamped_in_pose.header.frame_id = "camera3_frame";
                stamped_in_pose.header.stamp = this->get_clock()->now();
                stamped_in_pose.pose = part_pose.pose;
                geometry_msgs::msg::PoseStamped stamped_out_pose;
                stamped_out_pose = tf_buffer_->transform(stamped_in_pose, "map");
                int part_color = part_pose.part.color;
                int part_type = part_pose.part.type;
                std::string bat_color, bat_type;
                battery_data(part_color, part_type, bat_color, bat_type);
                RCLCPP_INFO(this->get_logger(), "Camera3 clor type detected: Color: %s, Type: %s", bat_color.c_str(), bat_type.c_str());
                PartInfo info;
                info.position = stamped_out_pose.pose.position;
                info.type = bat_type;  
                info.color = bat_color;
                cameraParts[2].push_back(info);
                RCLCPP_INFO(this->get_logger(), "Camera3 pose: x = %f, y = %f, z = %f",info.position.x, info.position.y, info.position.z);
            }
            to_stop = to_stop+1;
            RCLCPP_INFO(this->get_logger(), "cam3 subscriber stopped");
            cam3_sub.reset();
            if(to_stop==6){
                read_params();
            }
    }

    //Camera 4
    void GoalPublisher::cam4_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg) {
        cameraParts[3].clear();
            for (const auto &part_pose : msg->part_poses)
            {
                geometry_msgs::msg::PoseStamped stamped_in_pose;
                stamped_in_pose.header.frame_id = "camera4_frame";
                stamped_in_pose.header.stamp = this->get_clock()->now();
                stamped_in_pose.pose = part_pose.pose;
                geometry_msgs::msg::PoseStamped stamped_out_pose;
                stamped_out_pose = tf_buffer_->transform(stamped_in_pose, "map");
                int part_color = part_pose.part.color;
                int part_type = part_pose.part.type;
                std::string bat_color, bat_type;
                battery_data(part_color, part_type, bat_color, bat_type);
                RCLCPP_INFO(this->get_logger(), "Camera4 clor type detected: Color: %s, Type: %s", bat_color.c_str(), bat_type.c_str());
                PartInfo info;
                info.position = stamped_out_pose.pose.position;
                info.type = bat_type;  
                info.color = bat_color; 
                cameraParts[3].push_back(info);
                RCLCPP_INFO(this->get_logger(), "Camera4 pose: x = %f, y = %f, z = %f",info.position.x, info.position.y, info.position.z);
            }
            to_stop = to_stop+1;
            RCLCPP_INFO(this->get_logger(), "cam4 subscriber stopped");
            cam4_sub.reset();
            if(to_stop==6){
                read_params();
            }
    }

    //Camera 5
    void GoalPublisher::cam5_frame(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg) {
        cameraParts[4].clear();
            for (const auto &part_pose : msg->part_poses)
            {
                geometry_msgs::msg::PoseStamped stamped_in_pose;
                stamped_in_pose.header.frame_id = "camera5_frame";
                stamped_in_pose.header.stamp = this->get_clock()->now();
                stamped_in_pose.pose = part_pose.pose;
                geometry_msgs::msg::PoseStamped stamped_out_pose;
                stamped_out_pose = tf_buffer_->transform(stamped_in_pose, "map");
                int part_color = part_pose.part.color;
                int part_type = part_pose.part.type;
                std::string bat_color, bat_type;
                battery_data(part_color, part_type, bat_color, bat_type);
                RCLCPP_INFO(this->get_logger(), "Camera5 color type detected: Color: %s, Type: %s", bat_color.c_str(), bat_type.c_str());
                PartInfo info;
                info.position = stamped_out_pose.pose.position;
                info.type = bat_type;  
                info.color = bat_color; 
                cameraParts[4].push_back(info);
                RCLCPP_INFO(this->get_logger(), "Camera5 pose: x = %f, y = %f, z = %f",info.position.x, info.position.y, info.position.z);
            }
            to_stop = to_stop+1;
            RCLCPP_INFO(this->get_logger(), "cam5 subscriber stopped");
            cam5_sub.reset();
            if(to_stop==6){
                read_params();
            }
    }
    //Finds the object color and type 
    void GoalPublisher::battery_data(int part_color, int part_type, std::string& bat_color, std::string& bat_type) {
        switch (part_color) {
            case mage_msgs::msg::Part::BLUE:    bat_color = "BLUE"; break;
            case mage_msgs::msg::Part::GREEN:   bat_color = "GREEN"; break;
            case mage_msgs::msg::Part::ORANGE:  bat_color = "ORANGE"; break;
            case mage_msgs::msg::Part::RED:     bat_color = "RED"; break;
            case mage_msgs::msg::Part::PURPLE:  bat_color = "PURPLE"; break;
        }

        switch (part_type) {
            case mage_msgs::msg::Part::BATTERY:     bat_type = "BATTERY"; break;
            case mage_msgs::msg::Part::PUMP:        bat_type = "PUMP"; break;
            case mage_msgs::msg::Part::SENSOR:      bat_type = "SENSOR"; break;
            case mage_msgs::msg::Part::REGULATOR:   bat_type = "REGULATOR"; break;
        }
    }

    //Gets the bot current pose initialize it
    void GoalPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
        message.header.frame_id = "map";
        message.pose.pose.position.x = msg->pose.pose.position.x;
        message.pose.pose.position.y = msg->pose.pose.position.y;
        message.pose.pose.orientation.x = msg->pose.pose.orientation.x;
        message.pose.pose.orientation.y = msg->pose.pose.orientation.y;
        message.pose.pose.orientation.z = msg->pose.pose.orientation.z;
        message.pose.pose.orientation.w = msg->pose.pose.orientation.w;
        initial_pose_pub_->publish(message);
    }

    //Send goal to action server
    void GoalPublisher::send_goal()
    {
        using namespace std::placeholders;
        if (!this->client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(),
                        "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = FollowWaypoints::Goal();
        std::vector<std::array<double, 3>> points;

        std::array<double, 3> point;
        RCLCPP_INFO(this->get_logger(), "started");
        for (const auto& positions : allPositions) {
                for (const auto& pos : positions) {
                    point = {pos.x , pos.y, pos.z};
                    points.push_back(point);
                }
        }
        
        for (int i = 0; i < 5; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            std::array<double, 3> pose_a;
            pose_a = points[i];
            pose.header.frame_id = "map";
            pose.pose.position.x = pose_a[0];
            pose.pose.position.y = pose_a[1];
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.7787;
            goal_msg.poses.push_back(pose);            
        }

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options =
            rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&GoalPublisher::goal_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&GoalPublisher::result_callback, this, _1);
        client_->async_send_goal(goal_msg, send_goal_options);
    }

    //Server response for goal
    void GoalPublisher::goal_response_callback(
        std::shared_future<GoalHandleNavigation::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
            rclcpp::shutdown();
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(),
                        "Goal accepted, waiting for result");
        }
    }

    //server feedback response
    void GoalPublisher::feedback_callback(
        GoalHandleNavigation::SharedPtr,
        const std::shared_ptr<const FollowWaypoints::Feedback>)
    {
        RCLCPP_INFO(this->get_logger(), "Robot moving");
    }

    //server result response
    void GoalPublisher::result_callback(
        const GoalHandleNavigation::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_ERROR(this->get_logger(), "Goal was reached");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        rclcpp::shutdown();
    }   
}

int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<group16_final::GoalPublisher>("follow_waypoints");
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }