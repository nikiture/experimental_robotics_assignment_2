#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/* #include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h" */
#include "marker_search_interface/srv/marker_request.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;


class search_marker_action : public plansys2::ActionExecutorClient {
    public:
        search_marker_action ()
        :plansys2::ActionExecutorClient ("searh_marker_id", 250ms)
        {          
        }
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& previous_state)
        {
            mark_search_client = this->create_client<marker_search_interface::srv::MarkerRequest> (
                "marker_search_request"
            );
            auto req = std::make_shared<marker_search_interface::srv::MarkerRequest::Request>();
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
        rclcpp::Client<marker_search_interface::srv::MarkerRequest>::SharedPtr mark_search_client;
        void request_callback (
            rclcpp::Client<marker_search_interface::srv::MarkerRequest>::SharedFuture future)
        {
            if (std::future_status::ready == future.wait_for(250ms)) {
                auto res = future.get();
                if (res->found_marker) {
                    finish(true, 1.0, "Search completed");
                } else {
                    send_feedback (0.0, "search ongoing");
                }
            } else {
                RCLCPP_INFO (this->get_logger(), "waiting for server response");
            }
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