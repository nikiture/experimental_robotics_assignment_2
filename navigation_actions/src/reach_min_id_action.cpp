#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"


#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "marker_search_interface/srv/min_id_marker.hpp"
//#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class reach_min_id_action : public plansys2::ActionExecutorClient {
    public:
        reach_min_id_action ()
        :plansys2::ActionExecutorClient ("reach_min_id_marker", 250ms)
        {    
            
                 
        }
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& previous_state)
        {
            min_id_loc_client = this->create_client<marker_search_interface::srv::MinIdMarker> (
                "min_id_location"
            );
            //rotation_client->on_activate();
            //rotation_client = this->create_publisher<geometry_msgs::msg::Twist> ("/cmd_vel", 5);
            //timer = this-> create_wall_timer (100ms, std::bind (&reach_min_id_action::timer_callback, this));
            /*rotation_command.angular.z = 0.1;
            rotation_client->publish(rotation_command);*/
            /* auto search_wp = get_arguments()[2];
            auto search_loc = waypoints_[search_wp]; */
            /* auto wp_to_search = get_arguments()[2];  // The goal is in the 3rd argument of the action

            auto search_loc = waypoints_[wp_to_search]; */
            auto req = std::make_shared<marker_search_interface::srv::MinIdMarker::Request>();
            //req->curr_pos = search_loc;


            while (!min_id_loc_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "waiting for service");
            }
            auto request_future = min_id_loc_client->async_send_request(
                req, std::bind(&reach_min_id_action::request_callback, this, _1));

            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            pos_timer = this->create_wall_timer(500ms, std::bind(&reach_min_id_action::pos_callback, this));

            return ActionExecutorClient::on_activate (previous_state);
        }
    private:
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
        //std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
        marker_search_interface::srv::MinIdMarker::Request::SharedPtr req;
        rclcpp::TimerBase::SharedPtr timer;
        
        rclcpp::Client<marker_search_interface::srv::MinIdMarker>::SharedPtr min_id_loc_client;
        void request_callback (
            rclcpp::Client<marker_search_interface::srv::MinIdMarker>::SharedFuture future)
        {
            if (std::future_status::ready == future.wait_for(500ms)) {
                auto loc = future.get();
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


                navigation_goal.pose = loc->position;

                dist_to_move = getDistance(loc->position.pose, current_pos);

                /* auto send_goal_options =
                rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions(); */

                send_goal_options.feedback_callback = [this](
                NavigationGoalHandle::SharedPtr handle,
                NavigationFeedback feedback) {
                    progress = 1.0 - feedback->distance_remaining / dist_to_move;
                    /* if (progress > 0.96) {
                        navigation_action_client->async_cancel_goal(handle)
                        finish(true, 1.0, "Move completed");
                    } */
                    send_feedback(
                    std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
                    "Move running");
                };

            /* send_goal_options.result_callback = [this](auto) {
                navigation_action_client.reset();
                finish(true, 1.0, "Move completed");
                
            }; */
                send_goal_options.result_callback = std::bind(&reach_min_id_action::result_callback, this, _1);

                future_navigation_goal_handle =
                navigation_action_client->async_send_goal(navigation_goal, send_goal_options);
                } else {
                    RCLCPP_INFO (this->get_logger(), "waiting for server response");
                    /* rotation_command.angular.z = 0.1;
                    rotation_client->publish(rotation_command); */
                }
            }
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
        

};

int main (int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<reach_min_id_action>();

    node->set_parameter(rclcpp::Parameter("action_name", "reach_min_id_marker"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;

}