import rclpy
import rclpy.node
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
#from std_msgs.msg import Empty, Header
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from ros2_aruco import transformations
#from sensor_msgs.msg import CameraInfo
#from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import floor
from marker_search_interface.srv import MarkerRequest


class marker_searcher (rclpy.node.Node):
    def __init__(self):
        super().__init__("marker_search_server")
        
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)
        self.declare_parameter("circle_signal", "/place_circle")
        self.declare_parameter("marker_size", .0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")

        

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        
        dictionary_id_name = self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        
        #circle_topic = self.get_parameter("circle_signal").get_parameter_value().string_value
        
        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))
    

        


        self.info_sub = self.create_subscription(CameraInfo, 
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)

        self.create_subscription(Image, 
                                 image_topic,
                                 self.image_callback,
                                 qos_profile_sensor_data)
        
        self.search_serv = self.create_service (MarkerRequest, 'marker_search_request', self.search_callback)
        
        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.curr_image = None
        self.image_to_pub = None
        self.image_encoding = None
        
        
    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

   
    def image_callback (self, image_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return
        #if camera info available store image from camera for future processing
        self.curr_image = self.bridge.imgmsg_to_cv2(image_msg,
                                                    desired_encoding=image_msg.encoding)
        self.image_encoding = image_msg.encoding
        
        
    def search_callback (self, request, response):
        #find markers in last image received
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(self.curr_image,
                                                                self.aruco_dictionary,
                                                                parameters=self.aruco_parameters)
        """ if marker_ids is not None:
            #find center of first found marker: 
            # x position as average of corner 0 and corner 1 (along first direction),
            # y position as average of corner 0 and corner 3 (along second direction)
            self.get_logger().info("marker found!")
            response.found_marker = True
            response.ID = marker_ids[0]
        else:
            self.get_logger().info("no marker found")
            response.found_marker = False """
            
        response.found_marker = marker_ids is not None
        if response.found_marker:
            print (marker_ids [0][0])
            response.marker_id = int(marker_ids[0][0])
        
        return response
    
    
def main (args = None):
    rclpy.init(args=args)
    node = marker_searcher()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
                    
        