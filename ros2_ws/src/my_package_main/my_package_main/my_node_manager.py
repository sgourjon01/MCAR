##Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from dataclasses import dataclass
from typing import Dict
from queue import Queue
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from .robot_state import RobotState
from .map import obstacle
from .task_type import TaskType



##Classes pour l'orga des données
@dataclass
class RobotInfo:
    """Structure pour stocker les données de position d'un robot"""
    id: int
    state: int
    task: int
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    omega: float

@dataclass
class Task:
    """Structure pour les données de tâche à publier"""
    id: int
    robot: int
    state: int
    pickup_x: float
    pickup_y: float
    delivery_x: float
    delivery_y: float

    PENDING = 0
    ASSIGNED = 1
    IN_TRANSPORT = 2
    DONE = 3

    STR_STATE = ["Pending","Assigned","In progress","Done"]


##Classe MyManagerNode
class ManagerNode(Node):
    """Node Manager qui gère les subscriptions et publications"""
    
    def __init__(self):
        super().__init__('manager_node')
        
        # Dictionnaires pour stocker 4 robots
        self.robots: Dict[int, RobotInfo] = {}
        self.robots[1] = RobotInfo(1,RobotState.WAITING,-1,0,0,0,0,0,0)
        self.robots[2] = RobotInfo(2,RobotState.WAITING,-1,0,0,0,0,0,0)
        self.robots[3] = RobotInfo(3,RobotState.WAITING,-1,0,0,0,0,0,0)
        self.robots[4] = RobotInfo(4,RobotState.WAITING,-1,0,0,0,0,0,0)

        self.tasks: Dict[int, Task] = {}
        self.lock = threading.Lock()

        self.auto_tasks = False
        
        # Queue pour communiquer avec l'interface GUI de manière thread-safe
        self.ui_queue = Queue()
        
        # Subscriptions
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            'robot_pose',
            self.position_callback,
            10
        )
        
        self.etat_sub = self.create_subscription(
            Int32MultiArray,
            'robot_state',
            self.state_callback,
            10
        )
        
        # Publications
        self.task_pub = self.create_publisher(
            Float32MultiArray,
            'robot_task',
            10
        )
        
        #timer_period = 0.1  #100ms
        #self.timer = self.create_timer(timer_period, self.manage_tasks)
        
        self.get_logger().info('Manager Node initialized')
    
    def position_callback(self, msg: Float32MultiArray):
        """Callback pour le topic position
        Reçoit: [id, x, y, theta, vx, vy, omega]
        """
        with self.lock:
            try:
                if len(msg.data) != 7:
                    self.get_logger().error('Topic Pose : format incorrect')
                    return
                elif msg.data[0] < 1 or msg.data[0] > 4:
                    self.get_logger().error('Topic Pose : id robot incorrect')
                    return
                
                id_rob = int(msg.data[0])
                self.robots[id_rob].x = msg.data[1]
                self.robots[id_rob].y = msg.data[2]
                self.robots[id_rob].theta = msg.data[3]
                self.robots[id_rob].vx = msg.data[4]
                self.robots[id_rob].vy = msg.data[5]
                self.robots[id_rob].omega = msg.data[6]

                self.ui_queue.put(('position_update', self.robots[id_rob].id))

            except (IndexError, ValueError) as e:
                self.get_logger().error(f'Error parsing depuis pose : {e}')
    
    def state_callback(self, msg: Int32MultiArray):
        """Callback pour le topic etat
        Reçoit: [id, etat]
        """
        with self.lock:
            try:
                if len(msg.data) != 2:
                    self.get_logger().error('Topic State : format incorrect. Format attendu (int,int)')
                    return
                id_rob = msg.data[0]
                state_rob = msg.data[1]
                if id_rob < 1 or id_rob > 4:
                    self.get_logger().error('Topic State : Id robot invalide.')
                    return
                elif state_rob < RobotState.WAITING or state_rob > RobotState.TASK_DONE:
                    self.get_logger().error('Topic State : Etat invalide.')
                    return

                self.robots[id_rob].state = state_rob
                
                if state_rob == RobotState.TASK_DONE:
                    self.tasks[self.robots[id_rob].task].state = Task.DONE
                    self.publish_ack(id_rob)
                elif state_rob == RobotState.WAITING:
                    #if self.auto_tasks:
                        #create task random
                    next_pending = next((task for task in self.tasks.values() if task.state == Task.PENDING),None,)
                    if next_pending != None:
                        self.assign_task(next_pending.id)
                elif state_rob == RobotState.TRANSPORTING:
                    self.tasks[self.robots[id_rob].task].state = Task.IN_TRANSPORT
                
                self.ui_queue.put(('state_update', id_rob))

            except (IndexError, ValueError) as e:
                self.get_logger().error(f'Error parsing depuis topic State: {e}')

    def create_task(self, p_x:float, p_y:float, d_x:float,d_y:float):
        if obstacle(p_x,p_y) or obstacle(d_x,d_y):
            self.get_logger().error('Ne peut pas creer la tache, coordonnees incorrectes.')
            return

        task_id = max(self.tasks.keys(), default=-1) + 1
        self.tasks[task_id] = Task(id=task_id, robot=0, state=0, pickup_x=p_x, pickup_y=p_y, delivery_x=d_x, delivery_y=d_y)

        self.assign_task(task_id=task_id)
        self.ui_queue.put(('task_add', task_id))

    def assign_task(self, task_id):
        for robot in self.robots.values():
            if robot.state == RobotState.WAITING:
                self.tasks[task_id].robot = robot.id
                self.tasks[task_id].state = Task.ASSIGNED
                robot.task = task_id
                robot.state = RobotState.SEARCHING_PACKET #à modifier dans le futur pour update quand recoit topic
                self.publish_task(self.tasks[task_id])
                return

    def publish_task(self, task: Task):
        """Publication tâche sur topic"""
        try:
            msg = Float32MultiArray()
            msg.data = [
                float(task.id),
                float(TaskType.MOVE_PACKET),
                float(task.pickup_x),
                float(task.pickup_y),
                float(task.delivery_x),
                float(task.delivery_y)
            ]
            
            self.task_pub.publish(msg)
            self.get_logger().debug(f'Task published for robot {task.id}')
        except Exception as e:
            self.get_logger().error(f'Error publishing task: {e}')

    def publish_ack(self, id:int):
        try:
            msg = Float32MultiArray()
            msg.data = [
                float(id),
                float(TaskType.ACK)
            ]
            
            self.task_pub.publish(msg)
            self.get_logger().debug(f'Ack for robot {id}')
        except Exception as e:
            self.get_logger().error(f'Error publishing ack: {e}')

def run_ros(manager_node: ManagerNode):
    """Exécute ROS2 dans un thread séparé"""
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(manager_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        manager_node.get_logger().info('ROS2 thread shutting down...')

def main(args=None):
    rclpy.init(args=args)
    
    manager_node = ManagerNode()

    manager_node.create_task(200,300,400,400)

    ros_thread = threading.Thread(target=lambda: run_ros(manager_node), daemon=True)
    ros_thread.start()

    from .ui_manager import ManagerUI
    gui = ManagerUI(manager_node)
    
    gui.run()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
