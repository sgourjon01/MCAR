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



##Classes pour l'orga des données
@dataclass
class RobotInfo:
    """Structure pour stocker les données de position d'un robot"""
    id: int
    state: int
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
    IN_PROGRESS = 2
    DONE = 3


##Classe MyManagerNode
class ManagerNode(Node):
    """Node Manager qui gère les subscriptions et publications"""
    
    def __init__(self):
        super().__init__('manager_node')
        
        # Dictionnaires pour stocker jusqu'à 4 positions et états par ID
        self.robots: Dict[int, RobotInfo] = {}
        self.tasks: Dict[int, Task] = {}
        self.lock = threading.Lock()
        
        # Queue pour communiquer avec l'interface GUI de manière thread-safe
        self.ui_queue = Queue()
        
        # Subscriptions
        self.position_sub = self.create_subscription(
            Float32MultiArray,
            'position',
            self.position_callback,
            10
        )
        
        self.etat_sub = self.create_subscription(
            Int32MultiArray,
            'etat',
            self.state_callback,
            10
        )
        
        # Publications
        self.task_pub = self.create_publisher(
            Float32MultiArray,
            'task',
            10
        )
        
        # Timer pour appeler manage_tasks régulièrement
        timer_period = 0.1  # 100ms
        self.timer = self.create_timer(timer_period, self.manage_tasks)
        
        self.get_logger().info('Manager Node initialized')
        self.get_logger().info('Ready to manage up to 4 robots')
    
    def position_callback(self, msg: Float32MultiArray):
        """Callback pour le topic position
        Reçoit: [x, y, theta, vx, vy, omega, id]
        """
        with self.lock:
            try:
                if len(msg.data) < 7:
                    self.get_logger().error('Position message format incorrect')
                    return
                
                robot_pos = RobotInfo(
                    id=int(msg.data[6]),
                    x=float(msg.data[0]),
                    y=float(msg.data[1]),
                    theta=float(msg.data[2]),
                    vx=float(msg.data[3]),
                    vy=float(msg.data[4]),
                    omega=float(msg.data[5])
                )
                
                self.get_logger().info('Received "%s"' % msg.data)
                
                if len(self.robots) < 4 or robot_pos.id in self.robots:
                    self.robots[robot_pos.id] = robot_pos
                    self.get_logger().debug(
                        f'Position updated for robot {robot_pos.id}: '
                        f'({robot_pos.x:.2f}, {robot_pos.y:.2f})'
                    )
                    # Envoyer update à la GUI via queue
                    self.ui_queue.put(('position_update', robot_pos.id))
                else:
                    self.get_logger().warn(
                        f'Cannot store position for robot {robot_pos.id}: '
                        f'Maximum 4 robots reached'
                    )
            except (IndexError, ValueError) as e:
                self.get_logger().error(f'Error parsing position message: {e}')
    
    def state_callback(self, msg: Int32MultiArray):
        """Callback pour le topic etat
        Reçoit: [id, etat]
        """
        with self.lock:
            try:
                if len(msg.data) != 2:
                    self.get_logger().error('Topic State : format incorrect. Format attendu (int,int)')
                    return
                id_msg = msg.data[0]
                state_msg = msg.data[1]
                if id_msg < 1 or id_msg > 4:
                    self.get_logger().error('Topic State : Id robot invalide.')
                    return
                if state_msg < RobotState.WAIT or state_msg > RobotState.TASK_DONE:
                    self.get_logger().error('Topic State : Etat invalide.')
                    return

                self.robots[id_msg].state = state_msg
                
                self.ui_queue.put(('state_update', id_msg))

            except (IndexError, ValueError) as e:
                self.get_logger().error(f'Error parsing depuis topic State: {e}')

    def add_task(self, robot:int, p_x:float, p_y:float, d_x:float,d_y:float):
        task_id = max(self.tasks.keys(), default=-1) + 1
        self.tasks[task_id] = Task(id=task_id, robot=robot, state=0, pickup_x=p_x, pickup_y=p_y, delivery_x=d_x, delivery_y=d_y)

    
    def manage_tasks(self):
        """Fonction principale de traitement"""
        with self.lock:
            for robot_id in self.robots.keys():
                if robot_id in self.etats:
                    position = self.robots[robot_id]
                    etat = self.etats[robot_id]
                    
                    task = self.calculate_task(position, etat)
                    
                    if task:
                        self.publish_task(task)
    
    def calculate_task(self, position: RobotInfo) -> Task:
        """Calcule les tâches - À implémenter selon votre logique"""
        task = Task(
            id=position.id,
            pickup_x=position.x + 1.0,
            pickup_y=position.y + 1.0,
            delivery_x=position.x + 5.0,
            delivery_y=position.y + 5.0
        )
        
        return task
    
    def publish_task(self, task: Task):
        """Publie la tâche"""
        try:
            msg = Float32MultiArray()
            msg.data = [
                task.pickup_x,
                task.pickup_y,
                task.delivery_x,
                task.delivery_y,
                float(task.id)
            ]
            
            self.task_pub.publish(msg)
            self.get_logger().debug(f'Task published for robot {task.id}')
        except Exception as e:
            self.get_logger().error(f'Error publishing task: {e}')
    
    def get_position(self, robot_id: int) -> RobotInfo:
        """Récupère la position d'un robot par son ID"""
        with self.lock:
            return self.robots.get(robot_id)
    
    def get_all_robots(self) -> Dict[int, tuple]:
        """Retourne tous les robots avec leurs positions et états"""
        with self.lock:
            robots = {}
            for robot_id in self.robots.keys():
                if True:#robot_id in self.etats:
                    robots[robot_id] = (
                        self.robots[robot_id],
                        1#self.etats[robot_id]
                    )
            return robots

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

    manager_node.tasks[0] = Task(0,1,1,2.0,3.0,4.0,5.0)

    ros_thread = threading.Thread(target=lambda: run_ros(manager_node), daemon=True)
    ros_thread.start()

    from .ui_manager import ManagerUI
    gui = ManagerUI(manager_node)
    
    gui.run()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
