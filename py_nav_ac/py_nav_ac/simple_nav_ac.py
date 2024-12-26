import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose
#from tf_transformations import quaternion_from_euler
import math


class simple_nav_action_client (Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.nav_client = ActionClient (self, NavigateToPose, '/navigate_to_pose')
        
    def send_goal (self, des_pose):
        nav_msg = NavigateToPose.Goal()
        nav_msg.pose.pose = des_pose
        
        self.nav_client.wait_for_server()
        
        #return self.nav_client.send_goal_async (nav_msg)
        self.nav_future = self.nav_client.send_goal_async (nav_msg, feedback_callback = self.feedback_callback)
        
        self.nav_future.add_done_callback (self.accept_callback)
        
    def accept_callback (self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            return
        
        self.get_logger().info('goal accepted')
        
        self.future_result = goal_handle.get_result_async()
        self.future_result.add_done_callback(self.result_callback)
        
    def result_callback (self, future):
        result = future.result().result
        self.get_logger().info('result:{0}'.format(result))
        rclpy.shutdown()
        
    def feedback_callback (self, feedback_msg):
        self.get_logger().info('feedback: {0}'.format(feedback_msg.feedback))
        


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def main(args=None):
    #print('Hi from py_nav_ac.')
    rclpy.init(args=args)
    
    navig_client = simple_nav_action_client()
    
    des_pose = Pose()
    """ des_pose.position.x = 2.0
    des_pose.position.y = 1.5
    des_pose.orientation.z = 1.0 """
    #print("desired x position:\n")
    des_pose.position.x = float(input("desired x position:\n"))
    #print ("desired y position:\n")
    des_pose.position.y = float(input("desired y position:\n"))
    #print ("desired yaw")
    des_yaw = float(input ("desired yaw"))
    des_q = quaternion_from_euler (0, 0, des_yaw)
    des_pose.orientation.w = des_q [0]
    des_pose.orientation.x = des_q [1]
    des_pose.orientation.y = des_q [2]
    des_pose.orientation.z = des_q [3]
    
    print ("sending goal")
    
    future = navig_client.send_goal(des_pose)
    
    #rclpy.spin_until_future_complete(navig_client, future)
    
    rclpy.spin(navig_client)


if __name__ == '__main__':
    main()
