#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/* #include "nav2_msgs/action/navigate_to_pose.hpp"


#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h" */
#include "marker_search_interface/srv/marker_request.hpp"
#include "geometry_msgs/msg/twist.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;


class search_marker_action : public plansys2::ActionExecutorClient {
    public:
        search_marker_action ()
        :plansys2::ActionExecutorClient ("searh_marker_id", 250ms),
        rotation_command()
        {    
            geometry_msgs::msg::PoseStamped wp;
            wp.header.frame_id = "/map";
            wp.header.stamp = now(); 
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
                 
        }
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& previous_state)
        {
            mark_search_client = this->create_client<marker_search_interface::srv::MarkerRequest> (
                "marker_search_request"
            );
            //rotation_client->on_activate();
            //rotation_client = this->create_publisher<geometry_msgs::msg::Twist> ("/cmd_vel", 5);
            //timer = this-> create_wall_timer (100ms, std::bind (&search_marker_action::timer_callback, this));
            /*rotation_command.angular.z = 0.1;
            rotation_client->publish(rotation_command);*/
            /* auto search_wp = get_arguments()[2];
            auto search_loc = waypoints_[search_wp]; */
            auto wp_to_search = get_arguments()[2];  // The goal is in the 3rd argument of the action

            auto search_loc = waypoints_[wp_to_search];
            req = std::make_shared<marker_search_interface::srv::MarkerRequest::Request>();
            req->curr_pos = search_loc;


            while (!mark_search_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "waiting for service");
            }
            auto request_future = mark_search_client->async_send_request(
                req, std::bind(&search_marker_action::request_callback, this, _1));
            return ActionExecutorClient::on_activate (previous_state);
        }
    private:
    std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
        marker_search_interface::srv::MarkerRequest::Request::SharedPtr req;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotation_client;
        geometry_msgs::msg::Twist rotation_command;
        rclcpp::Client<marker_search_interface::srv::MarkerRequest>::SharedPtr mark_search_client;
        void request_callback (
            rclcpp::Client<marker_search_interface::srv::MarkerRequest>::SharedFuture future)
        {
            if (std::future_status::ready == future.wait_for(500ms)) {
                auto res = future.get();
                if (res->found_marker) {
                    RCLCPP_INFO(this->get_logger(), "found marker %d", res->marker_id);
                    //rotation_command.angular.z = 0.0;
                    //rotation_client->publish(rotation_command);
                    //timer->reset();
                    finish(true, 1.0, "Search completed");
                } else {
                    /* rotation_command.angular.z = 0.1;
                    rotation_client->publish(rotation_command); */
                    send_feedback (0.0, "search ongoing");
                    auto request_future = mark_search_client->async_send_request(
                        req, std::bind(&search_marker_action::request_callback, this, _1));

                }
            } else {
                RCLCPP_INFO (this->get_logger(), "waiting for server response");
                /* rotation_command.angular.z = 0.1;
                rotation_client->publish(rotation_command); */
            }
        }
        void timer_callback () {
            rotation_command.angular.z = 0.1;
            //rotation_client->publish(rotation_command);
        }

};

int main (int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<search_marker_action>();

    node->set_parameter(rclcpp::Parameter("action_name", "search_marker_id"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;

}
