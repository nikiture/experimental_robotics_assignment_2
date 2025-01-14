#include <memory>
#include <algorithm>
#include <cmath>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class robot_move_node:public plansys2::ActionExecutorClient {
    public:
        robot_move_node(/* args */) 
        :plansys2::ActionExecutorClient ("robot_move", 250ms),
        robot_frame("link_chassis"),
        map_frame ("map")
        {
            geometry_msgs::msg::PoseStamped wp;
            wp.header.frame_id = "/map";
            wp.header.stamp = now();
            /* wp.pose.position.x = -7.0;
            wp.pose.position.y = -1.5;
            wp.pose.position.z = 0.0;
            wp.pose.orientation.x = 0.0;
            wp.pose.orientation.y = 0.0;
            wp.pose.orientation.z = 0.0;
            wp.pose.orientation.w = 1.0;
            waypoints_["l1"] = wp;

            wp.pose.position.x = -3.0;
            wp.pose.position.y = -8.0;
            waypoints_["l2"] = wp;

            wp.pose.position.x = 6.0;
            wp.pose.position.y = 2.0;
            waypoints_["l3"] = wp;

            wp.pose.position.x = 7.0;
            wp.pose.position.y = -5.0;
            waypoints_["l4"] = wp; */
            wp.pose.orientation.x = 0.0;
            wp.pose.orientation.y = 0.0;
            wp.pose.orientation.z = 0.0;
            wp.pose.orientation.w = 1.0;
            wp.pose.position.x = -6.5;
            wp.pose.position.y = 1.5;
            double yaw = -2.0;

            /* wp.pose.orientation.z = std::sin(yaw/2);
            wp.pose.orientation.w = std::cos(yaw/2); */
            waypoints_["l1"] = wp;

            yaw = 3.14;
            wp.pose.position.x = -3.5;
            wp.pose.position.y = -8.0;
            /* wp.pose.orientation.z = std::sin(yaw/2);
            wp.pose.orientation.w = std::cos(yaw/2); */
            waypoints_["l2"] = wp;


            yaw = 2.0;
            wp.pose.position.x = 6.0;
            wp.pose.position.y = 2.0;
            /* wp.pose.orientation.z = std::sin(yaw/2);
            wp.pose.orientation.w = std::cos(yaw/2); */
            waypoints_["l3"] = wp;

            yaw = 1.54;
            wp.pose.position.x = 7.0;
            wp.pose.position.y = -5.5;
            /* wp.pose.orientation.z = std::sin(yaw/2);
            wp.pose.orientation.w = std::cos(yaw/2); */
            waypoints_["l4"] = wp;

            /* wp.pose.position.x = -2.0;
            wp.pose.position.y = -0.4;
            waypoints_["wp_control"] = wp; */

            /* using namespace std::placeholders;
            pos_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose",
            10,
            std::bind(&MoveAction::current_pos_callback, this, _1)); */
            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            pos_timer = this->create_wall_timer(500ms, std::bind(&robot_move_node::pos_callback, this));
        }

        /* void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            current_pos_ = msg->pose.pose;
        } */
        void pos_callback () {
            geometry_msgs::msg::TransformStamped t;

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
            try {
                t = tf_buffer->lookupTransform(
                robot_frame, map_frame,
                tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                robot_frame.c_str(), map_frame.c_str(), ex.what());
                return;
            }
            auto t_pos = geometry_msgs::msg::Pose ();
            t_pos.position.x = t.transform.translation.x;
            t_pos.position.y = t.transform.translation.y;
            t_pos.position.z = t.transform.translation.z;
            t_pos.orientation = t.transform.rotation;
            current_pos = t_pos;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) {
            send_feedback(0.0, "Move starting");
            progress = 0;
            navigation_action_client =
            rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                shared_from_this(),
                "navigate_to_pose");

            bool is_action_server_ready = false;
            do {
                RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

                is_action_server_ready =
                    navigation_action_client->wait_for_action_server(std::chrono::seconds(5));
            } while (!is_action_server_ready);

            RCLCPP_INFO(get_logger(), "Navigation action server ready");

            auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
            RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

            goal_pos = waypoints_[wp_to_navigate];
            navigation_goal.pose = goal_pos;

            dist_to_move = getDistance(goal_pos.pose, current_pos);

            /* auto send_goal_options =
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions(); */

            send_goal_options.feedback_callback = [this](
            NavigationGoalHandle::SharedPtr handle,
            NavigationFeedback feedback) {
                progress = 1.0 - feedback->distance_remaining / dist_to_move;
                if (progress > 0.96) {
                    navigation_action_client->async_cancel_goal(handle);
                    finish(true, 1.0, "Move completed");
                } else {
                send_feedback(
		        std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
		        "Move running");
                }
            };

            /* send_goal_options.result_callback = [this](auto) {
                navigation_action_client.reset();
                finish(true, 1.0, "Move completed");
                
            }; */
            send_goal_options.result_callback = std::bind(&robot_move_node::result_callback, this, _1);

            future_navigation_goal_handle =
            navigation_action_client->async_send_goal(navigation_goal, send_goal_options);

            return ActionExecutorClient::on_activate(previous_state);
    }
        //~robot_move_node();

    private:
        double getDistance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2) {
            return sqrt(
                (pos1.position.x - pos2.position.x) * (pos1.position.x - pos2.position.x) +
                (pos1.position.y - pos2.position.y) * (pos1.position.y - pos2.position.y));
        }

        void do_work(){
        }

        void result_callback (const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                finish(true, 1.0, "Move completed");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                if (progress > 0.97) {
                    finish(true, 1.0, "Move completed");
                    return;
                }
                RCLCPP_ERROR(this->get_logger(), "action aborted, retrying to send it");
                future_navigation_goal_handle =
                    navigation_action_client->async_send_goal(navigation_goal, send_goal_options);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "action canceled");
                finish(false, 0.0, "action cancelled");
                //return;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "unknown result code");
                finish(false, 0.0, "unknown action result");
                return;
            }
        }
        double progress;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
        std::string robot_frame;
        std::string map_frame;
        rclcpp::TimerBase::SharedPtr pos_timer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;

        using NavigationGoalHandle =
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
        using NavigationFeedback =
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client;
        std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle;
        NavigationGoalHandle::SharedPtr navigation_goal_handle;

        //rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub;
        geometry_msgs::msg::Pose current_pos;
        geometry_msgs::msg::PoseStamped goal_pos;
        nav2_msgs::action::NavigateToPose::Goal navigation_goal;

        double dist_to_move;

};




int main (int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_move_node>();

    node->set_parameter(rclcpp::Parameter("action_name", "robot_move"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}



