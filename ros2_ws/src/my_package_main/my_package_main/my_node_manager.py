##Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from dataclasses import dataclass
from typing import Dict
from queue import Queue
from std_msgs.msg import Float32MultiArray, Int32MultiArray

from ui_manager import ManagerUI


##Classes pour l'orga des données
@dataclass
class RobotPosition:
    """Structure pour stocker les données de position d'un robot"""
    id: int
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    omega: float

@dataclass
class RobotEtat:
    """Structure pour stocker l'état d'un robot"""
    id: int
    etat: int

@dataclass
class TaskData:
    """Structure pour les données de tâche à publier"""
    id: int
    pickup_x: float
    pickup_y: float
    delivery_x: float
    delivery_y: float


##Classe MyManagerNode
class ManagerNode(Node):
    """Node Manager qui gère les subscriptions et publications"""
    
    def __init__(self):
        super().__init__('manager_node')
        
        # Dictionnaires pour stocker jusqu'à 4 positions et états par ID
        self.positions: Dict[int, RobotPosition] = {}
        self.etats: Dict[int, RobotEtat] = {}
        self.tasks: Dict[int, TaskData] = {}
        self.lock = threading.Lock()
        
        # Queue pour communiquer avec l'interface GUI de manière thread-safe
        self.gui_update_queue = Queue()
        
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
            self.etat_callback,
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
                
                robot_pos = RobotPosition(
                    id=int(msg.data[6]),
                    x=float(msg.data[0]),
                    y=float(msg.data[1]),
                    theta=float(msg.data[2]),
                    vx=float(msg.data[3]),
                    vy=float(msg.data[4]),
                    omega=float(msg.data[5])
                )
                
                self.get_logger().info('Received "%s"' % msg.data)
                
                if len(self.positions) < 4 or robot_pos.id in self.positions:
                    self.positions[robot_pos.id] = robot_pos
                    self.get_logger().debug(
                        f'Position updated for robot {robot_pos.id}: '
                        f'({robot_pos.x:.2f}, {robot_pos.y:.2f})'
                    )
                    # Envoyer update à la GUI via queue
                    self.gui_update_queue.put(('position_update', robot_pos.id))
                else:
                    self.get_logger().warn(
                        f'Cannot store position for robot {robot_pos.id}: '
                        f'Maximum 4 robots reached'
                    )
            except (IndexError, ValueError) as e:
                self.get_logger().error(f'Error parsing position message: {e}')
    
    def etat_callback(self, msg: Int32MultiArray):
        """Callback pour le topic etat
        Reçoit: [etat, id]
        """
        with self.lock:
            try:
                if len(msg.data) < 2:
                    self.get_logger().error('Etat message format incorrect')
                    return
                
                robot_etat = RobotEtat(
                    id=msg.data[1],
                    etat=msg.data[0]
                )
                
                if len(self.etats) < 4 or robot_etat.id in self.etats:
                    self.etats[robot_etat.id] = robot_etat
                    self.get_logger().debug(
                        f'Etat updated for robot {robot_etat.id}: {robot_etat.etat}'
                    )
                    # Envoyer update à la GUI via queue
                    self.gui_update_queue.put(('etat_update', robot_etat.id))
                else:
                    self.get_logger().warn(
                        f'Cannot store etat for robot {robot_etat.id}: '
                        f'Maximum 4 robots reached'
                    )
            except (IndexError, ValueError) as e:
                self.get_logger().error(f'Error parsing etat message: {e}')
    
    def manage_tasks(self):
        """Fonction principale de traitement"""
        with self.lock:
            for robot_id in self.positions.keys():
                if robot_id in self.etats:
                    position = self.positions[robot_id]
                    etat = self.etats[robot_id]
                    
                    task = self.calculate_task(position, etat)
                    
                    if task:
                        self.publish_task(task)
    
    def calculate_task(self, position: RobotPosition, etat: RobotEtat) -> TaskData:
        """Calcule les tâches - À implémenter selon votre logique"""
        task = TaskData(
            id=position.id,
            pickup_x=position.x + 1.0,
            pickup_y=position.y + 1.0,
            delivery_x=position.x + 5.0,
            delivery_y=position.y + 5.0
        )
        
        return task
    
    def publish_task(self, task: TaskData):
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
    
    def get_position(self, robot_id: int) -> RobotPosition:
        """Récupère la position d'un robot par son ID"""
        with self.lock:
            return self.positions.get(robot_id)
    
    def get_etat(self, robot_id: int) -> RobotEtat:
        """Récupère l'état d'un robot par son ID"""
        with self.lock:
            return self.etats.get(robot_id)
    
    def get_all_robots(self) -> Dict[int, tuple]:
        """Retourne tous les robots avec leurs positions et états"""
        with self.lock:
            robots = {}
            for robot_id in self.positions.keys():
                if True:#robot_id in self.etats:
                    robots[robot_id] = (
                        self.positions[robot_id],
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
    
    # Créer le node
    manager_node = ManagerNode()
    
    # Créer la GUI dans le thread principal
    gui = ManagerUI(manager_node)
    
    # Lancer ROS2 dans un thread séparé
    ros_thread = threading.Thread(target=lambda: run_ros(manager_node), daemon=False)
    ros_thread.start()
    
    # Lancer la boucle tkinter dans le thread principal
    # ✅ CECI EST LA SOLUTION : mainloop dans le thread principal
    gui.run()
    
    # Cleanup
    rclpy.shutdown()


if __name__ == '__main__':
    main()
