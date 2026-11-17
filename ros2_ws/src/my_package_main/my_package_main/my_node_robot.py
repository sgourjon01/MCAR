import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray
import sys


class RobotNode(Node):
    """
    Classe qui gère un robot.
    """

    def __init__(self, robot_id):
        super().__init__("Robot_"+robot_id)
        self.robot_id = robot_id
        self.name = "Robot_"+robot_id
        
        # Constantes du robot
        self.r = 91 * 1e-3 # 91 mm

        # Subscriber task
        self.task_sub = self.create_subscription(Float32MultiArray,"/robot_task", self.listener_task, 10)
        
        # Subscriber pose
        self.pose_sub = self.create_subscription(Float32MultiArray,"/robot_pose", self.listener_pose, 10)

        # Publisher state
        self.state_pub = self.create_publisher(Int8MultiArray, "/robot_state", 10)
        
        # Main loop
        loop_timer_period = 0.1  # secondes
        self.timer = self.create_timer(loop_timer_period, self.loop)
        
        self.get_logger().info("Lancement du noeud pour le robot "+self.name+".")

    def listener_task(self, msg):
        """
        Subscriber task
        Traitement des ordes du Manager.
        """
        
        # TODO
        pass

    def listener_pose(self, msg):
        """
        Subscriber pose
        Actualisation des variables du robot.
        """
        
        # TODO
        pass
    
    def loop(self):
        """
        Main loop
        """

        # TODO
        pass
    

def main(args=None):
    robot_id = str(sys.argv[1]) if len(sys.argv) > 1 else "None"
    
    rclpy.init(args=args)
    node = RobotNode(robot_id)
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
