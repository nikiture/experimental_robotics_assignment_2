import rclpy
#from rclpy.action import ActionClient
from rclpy.node import Node
from plansys2_msgs.srv import AddProblem

import sys



class problem_loader (Node):
    def __init__(self):
        super().__init__('problem_loader')
        self.load_client = self.create_client (AddProblem, '/problem_expert/addproblem')
        while not self.load_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        self.load_req = AddProblem.request()
        
    def send_request (self, problem_file_path):
        problem_file = open(problem_file_path, 'r')
        problem_string = problem_file.read()
        self.load_req.problem = problem_string
        self.load_future = self.load_client.call_async(self.load_req)
        rclpy.spin_until_future_complete(self, self.load_future)
        return self.future.result()
    
def main (args=None):
    rclpy.init(args=args)
    
    problem_client = problem_loader()
    load_res = problem_client.send_request(sys.argv[1])
    problem_client.get_logger().info('results of problem loading:')
    if load_res.success:
        problem_client.get_logger().info('load successful')
    else:
        problem_client.get_logger().info('load unsuccessful, error : {}'.format(load_res.error_info))
        
    problem_client.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

    
