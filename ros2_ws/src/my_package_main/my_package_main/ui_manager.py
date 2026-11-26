from .my_node_manager import ManagerNode
from .my_node_manager import Task, RobotInfo
from .map import Map
from .robot_state import RobotState

from math import cos, sin
import queue
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import Float32MultiArray #A enlever par la suite, sert pour les tests

ROBOT_RADIUS = 10 #mm
PX_PAR_MM = 1.0
ROBOT_COLORS = ["black","DarkOrchid1","DarkOrange2","DarkSeaGreen3","cyan3"]
STATE_COLORS = ["white", "red", "gold2", "green", "blue"]

def parse_vector(text):
    """Convertit 'x;y' en (float, float). ValueError si le format est invalide."""
    x, y = text.split(";")
    return float(x), float(y)

class ManagerUI:
    """Interface graphique (thread principal)"""
    
    def __init__(self, node: ManagerNode):
        self.manager_node = node
        self.root = tk.Tk()
        self.root.title('Fleet Manager Interface')
        self.root.geometry('960x540+0+0')
        
        self.setup_ui()
        
        self.update_display()
        
        # Traiter les updates de la queue tous les 100ms
        self.process_queue()

    ##Création des éléments de l'interface
    def setup_ui(self):
        #Menu
        mainmenu = tk.Menu(self.root)
        mainmenu.add_command(label = "Map", command=self.setup_map_window)
        mainmenu.add_command(label = "Exit", command= self.root.destroy)
        self.root.config(menu = mainmenu)

        # Taches
        tk.Label(self.root, text="Tasks", font=("Arial", 16, "bold")).pack(pady=10)
        columns = ("task_id", "robot_id", "state", "pick_up", "delivery")
        headers = ("Task id", "Robot id", "State", "Pick up", "Delivery")
        self.tree = ttk.Treeview(self.root, columns=columns, show="headings", height=8)
        for col, text in zip(columns, headers):
            self.tree.heading(col, text=text)
            self.tree.column(col, width=130, anchor="center")
        self.tree.pack(fill="x", padx=10)

        # Robots
        tk.Label(self.root, text="Robots", font=("Arial", 16, "bold")).pack(pady=10)
        self.robots_grid = tk.Frame(self.root)
        self.robots_grid.pack(fill="x")
        tk.Label(self.robots_grid, text="Robot", font=("Arial", 12, "bold")).grid(row=0,column=0,padx=10)
        tk.Label(self.robots_grid, text="State", font=("Arial", 12, "bold")).grid(row=0,column=1,padx=10)
        tk.Label(self.robots_grid, text="Task", font=("Arial", 12, "bold")).grid(row=0,column=2,padx=10)
        tk.Label(self.robots_grid, text="Position", font=("Arial", 12, "bold")).grid(row=0,column=3, padx=10)
        tk.Label(self.robots_grid, text="Velocity", font=("Arial", 12, "bold")).grid(row=0,column=4,padx=10)

        self.rbt_lbls = {}

        for i in range(1,5):
            z1 = tk.Label(self.robots_grid, text="id", justify="left")
            z1.grid(row=i, column=0, padx=10)
            z2 = tk.Label(self.robots_grid, text="state", justify="left")
            z2.grid(row=i, column=1, padx=10)
            z3 = tk.Label(self.robots_grid, text="task", justify="left")
            z3.grid(row=i, column=2, padx=10)
            z4 = tk.Label(self.robots_grid, text="position", justify="left")
            z4.grid(row=i, column=3, padx=10)
            z5 = tk.Label(self.robots_grid, text="speed", justify="left")
            z5.grid(row=i, column=4, padx=10)

            self.rbt_lbls[(i,0)] = z1
            self.rbt_lbls[(i,1)] = z2
            self.rbt_lbls[(i,2)] = z3
            self.rbt_lbls[(i,3)] = z4
            self.rbt_lbls[(i,4)] = z5

        # Setup la map des le depart
        self.setup_map_window()


    pos = 200
    rad = 0

    def setup_map_window(self):
        if (getattr(self, "canvas", None) is not None):
            if self.canvas.winfo_exists():
                return
            
        win = tk.Toplevel(self.root)
        win.title("Warehouse Map")
        win.geometry(str(int(Map.X*Map.SIZE*PX_PAR_MM)+360)+"x"+str(int(Map.Y*Map.SIZE*PX_PAR_MM))+"+960+0")

        # Map container
        map_container = tk.Frame(win)
        map_container.pack(side="top", fill="x")

        # Map
        self.canvas = tk.Canvas(map_container, bg="white",width=Map.X*Map.SIZE*PX_PAR_MM,height=Map.Y*Map.SIZE*PX_PAR_MM)
        self.canvas.pack(side="left")

        # Options
        ##Affichage
        tk.Label(map_container, text="Options", font=("Arial", 12, "bold")).pack()
        tk.Label(map_container, text="\nDisplay", font=("Arial", 12, "bold")).pack(anchor="w")
        self.display_pending = tk.BooleanVar(value=True)
        self.display_assigned = tk.BooleanVar(value=True)
        self.display_progressing = tk.BooleanVar(value=False)
        self.display_done = tk.BooleanVar(value=False)
        tk.Checkbutton(map_container, text="Tasks pending", variable=self.display_pending).pack(anchor="w")
        tk.Checkbutton(map_container, text="Tasks being picked up", variable=self.display_assigned).pack(anchor="w")
        tk.Checkbutton(map_container, text="Tasks being delivered", variable=self.display_progressing).pack(anchor="w")
        tk.Checkbutton(map_container, text="Tasks done", variable=self.display_done).pack(anchor="w")

        ##Taches
        tk.Label(map_container, text="\nTasks", font=("Arial", 12, "bold")).pack(anchor="w")
        task_info = tk.Label(map_container, text=f"Task creation", justify="left")
        task_info.pack(anchor="w")
        #
        pckp = [-1,-1]
        delv = [-1,-1]
        def set_task_info_text():
            p1 = "Pickup (shift+mouse):"
            p2 = "Delivery (alt+mouse):"
            if pckp[0] > 0:
                p1 = f"Pickup (shift+mouse): {pckp[0]} ; {pckp[1]}"
            if delv[0] > 0:
                p2 = f"Delivery (alt+mouse): {delv[0]} ; {delv[1]}"
            task_info.config(text="Create a task (mouse click ou alt/shift + drag) :\n" + p1 + "\n" + p2)
        set_task_info_text()
        #
        def submit_task():
            with self.manager_node.lock:
                self.manager_node.create_task(p_x=pckp[0],p_y=pckp[1],d_x=delv[0],d_y=delv[1])
            pckp[0], pckp[1] = -1, -1
            delv[0], delv[1] = -1, -1
            set_task_info_text()
            self.canvas.delete("tmp_task")
        tk.Button(map_container, text="Submit Task", command=submit_task).pack(anchor="w")
        #
        def set_auto_tasks():
            self.manager_node.auto_tasks = auto_tasks.get()
        #
        auto_tasks = tk.BooleanVar(value=False)
        tk.Checkbutton(map_container, text="Auto create tasks", variable=auto_tasks, command=set_auto_tasks).pack(anchor="w")


        ###Obstacles
        obstacle_size_px = Map.SIZE * PX_PAR_MM
        for x in range(Map.Y):
            for y in range(Map.X):
                if Map.MAP_TABLE[y,x] == 1:
                    self.canvas.create_rectangle(x*obstacle_size_px, y*obstacle_size_px, (x+1)*obstacle_size_px, (y+1)*obstacle_size_px,outline="black",fill="grey")

        ###Robots
        rob_diam_px = 2 * ROBOT_RADIUS * PX_PAR_MM
        self.canvas.create_oval(100, 100, 100+rob_diam_px, 100+rob_diam_px, outline=STATE_COLORS[0], fill=ROBOT_COLORS[1], tags="c1", width=2)
        self.canvas.create_oval(200, 200, 200+rob_diam_px, 200+rob_diam_px, outline=STATE_COLORS[0], fill=ROBOT_COLORS[2], tags="c2", width=2)
        self.canvas.create_oval(300, 300, 300+rob_diam_px, 300+rob_diam_px, outline=STATE_COLORS[0], fill=ROBOT_COLORS[3], tags="c3", width=2)
        self.canvas.create_oval(400, 400, 400+rob_diam_px, 400+rob_diam_px, outline=STATE_COLORS[0], fill=ROBOT_COLORS[4], tags="c4", width=2)
    
        self.canvas.create_line(100, 100, 100 - ROBOT_RADIUS * PX_PAR_MM, 100 - ROBOT_RADIUS * PX_PAR_MM, fill=STATE_COLORS[0], tags="l1", width=2)
        self.canvas.create_line(100, 100, 100 - ROBOT_RADIUS * PX_PAR_MM, 100 - ROBOT_RADIUS * PX_PAR_MM, fill=STATE_COLORS[0], tags="l2", width=2)
        self.canvas.create_line(100, 100, 100 - ROBOT_RADIUS * PX_PAR_MM, 100 - ROBOT_RADIUS * PX_PAR_MM, fill=STATE_COLORS[0], tags="l3", width=2)
        self.canvas.create_line(100, 100, 100 - ROBOT_RADIUS * PX_PAR_MM, 100 - ROBOT_RADIUS * PX_PAR_MM, fill=STATE_COLORS[0], tags="l4", width=2)

        self.canvas.create_text(100, 100, text="1", tags="t1", font=("Old English Text MT",13,"bold italic"), fill=ROBOT_COLORS[1])
        self.canvas.create_text(100, 100, text="2", tags="t2", font=("Old English Text MT",13,"bold italic"), fill=ROBOT_COLORS[2])
        self.canvas.create_text(100, 100, text="3", tags="t3", font=("Old English Text MT",13,"bold italic"), fill=ROBOT_COLORS[3])
        self.canvas.create_text(100, 100, text="4", tags="t4", font=("Old English Text MT",13,"bold italic"), fill=ROBOT_COLORS[4])

        ##Legende
        #"■"
        tk.Label(map_container, text="\nLégende", font=("Arial", 13, "bold")).pack(anchor="w")
        tk.Label(map_container, text="Robot state (outline) :").pack(anchor="w")
        tk.Label(map_container, text="■ : "+RobotState.STR_STATE[0], fg=STATE_COLORS[0]).pack(anchor="w")
        tk.Label(map_container, text="■ : "+RobotState.STR_STATE[1], fg=STATE_COLORS[1]).pack(anchor="w")
        tk.Label(map_container, text="■ : "+RobotState.STR_STATE[2], fg=STATE_COLORS[2]).pack(anchor="w")
        tk.Label(map_container, text="■ : "+RobotState.STR_STATE[3], fg=STATE_COLORS[3]).pack(anchor="w")
        tk.Label(map_container, text="■ : "+RobotState.STR_STATE[4], fg=STATE_COLORS[4]).pack(anchor="w")
        ##Tests
        tk.Label(map_container, text="\nTest", font=("Arial", 12, "bold")).pack(anchor="w")
        #
        def test_move():
            msg = Float32MultiArray()
            self.pos += 10
            self.rad += 0.1
            #[id, x, y, theta, vx, vy, omega]
            msg.data = [
                float(1),
                float(self.pos),
                float(self.pos),
                float(self.rad),
                float(0),
                float(0),
                float(0),]
            print("test " + str(list(msg.data)))
            self.manager_node.position_callback(msg=msg)
        #
        tk.Button(map_container, text="Move robot", command=test_move).pack(anchor="w")

        def on_drag_pickup(event):
            pckp[0], pckp[1] = event.x/PX_PAR_MM, event.y/PX_PAR_MM
            set_task_info_text()
            if pckp[0] > 0 and delv[0] > 0:
                self.canvas.delete("tmp_task")
                self.canvas.create_line(pckp[0] * PX_PAR_MM, pckp[1] * PX_PAR_MM, delv[0] * PX_PAR_MM, delv[1] * PX_PAR_MM, fill="crimson", tags="tmp_task", arrow="last")

        def on_drag_delivery(event):
            delv[0], delv[1] = event.x/PX_PAR_MM, event.y/PX_PAR_MM
            set_task_info_text()
            if pckp[0] > 0 and delv[0] > 0:
                self.canvas.delete("tmp_task")
                self.canvas.create_line(pckp[0] * PX_PAR_MM, pckp[1] * PX_PAR_MM, delv[0] * PX_PAR_MM, delv[1] * PX_PAR_MM, fill="crimson", tags="tmp_task", arrow="last")

        self.canvas.bind("<Shift-Motion>", on_drag_pickup)
        self.canvas.bind("<Alt-Motion>", on_drag_delivery)
        self.canvas.bind("<Button-1>", on_drag_pickup)
        self.canvas.bind("<B1-Motion>", on_drag_delivery)

        self.update_display()
    
    def update_display(self):
        """Met à jour l'affichage"""
        with self.manager_node.lock:
            all_tasks = list(self.manager_node.tasks.values())
            robots = list(self.manager_node.robots.values())

        self.tree.delete(*self.tree.get_children())
        self.canvas.delete("task")
        task_display_state = (self.display_pending.get() << 0) | (self.display_assigned.get() << 1) | (self.display_progressing.get() << 2) | (self.display_done.get() << 3)
        for task in all_tasks:
            self.tree.insert("", "end",
                             values=(task.id, task.robot, Task.STR_STATE[task.state], f"({task.pickup_x};{task.pickup_y})",f"({task.delivery_x};{task.delivery_y})",))
            if (task_display_state & (1 << (task.state))) != 0: #(self.display_pending.get() and task.state == Task.PENDING) or (self.display_assigned.get() and task.state == Task.ASSIGNED) or (self.display_progressing.get() and task.state == Task.IN_PROGRESS) or (self.display_done.get() and task.state == Task.DONE):
                self.draw_task(task=task)

        for r in robots:
            self.move_robot(r, r.id)
            self.canvas.itemconfig("c" + str(r.id), outline=STATE_COLORS[r.state])
            self.canvas.itemconfig("l" + str(r.id), fill=STATE_COLORS[r.state])

            self.rbt_lbls[(r.id,0)].config(text=r.id, fg=ROBOT_COLORS[r.id], font=("Arial",10,"bold"))
            self.rbt_lbls[(r.id,1)].config(text=RobotState.STR_STATE[r.state], fg=STATE_COLORS[r.state])
            self.rbt_lbls[(r.id,2)].config(text=r.task)
            self.rbt_lbls[(r.id,3)].config(text=f"({round(r.x)} ; {round(r.y)} ) mm\n{round(r.theta,2)} rad")
            self.rbt_lbls[(r.id,4)].config(text=f"({round(r.vx)} ; {round(r.vy)} ) mm/s\n{round(r.omega,2)} rad/s")

            print(f"{r.id} : {all_tasks[r.task].id} ({r.task})")
        print(all_tasks)

    def draw_task(self, task:Task):
        self.canvas.create_line(task.pickup_x * PX_PAR_MM, task.pickup_y * PX_PAR_MM, task.delivery_x * PX_PAR_MM, task.delivery_y * PX_PAR_MM, fill=ROBOT_COLORS[task.robot], tags="task", arrow="last")
        self.canvas.create_text(task.pickup_x * PX_PAR_MM, task.pickup_y * PX_PAR_MM, text=str(task.id), tags="task", font=("Arial",10,"normal"), fill=ROBOT_COLORS[task.robot], anchor="nw")
    
    def move_robot(self, robot:RobotInfo, tag:int):
        self.canvas.moveto("c" + str(tag),(robot.x - ROBOT_RADIUS) * PX_PAR_MM, (robot.y - ROBOT_RADIUS) * PX_PAR_MM)
        self.canvas.coords("l"+str(tag), robot.x * PX_PAR_MM,
                        robot.x * PX_PAR_MM,
                        (robot.x - ROBOT_RADIUS * cos(robot.theta)) * PX_PAR_MM,
                        (robot.x + ROBOT_RADIUS * sin(robot.theta)) * PX_PAR_MM)
        self.canvas.moveto("t"+str(tag), robot.x + ROBOT_RADIUS, robot.y + ROBOT_RADIUS)
    
    def process_queue(self):
        """Traite les messages de la queue ROS2"""
        try:
            msg_type, robot_id = self.manager_node.ui_queue.get_nowait()
            self.update_display()
        except queue.Empty:
            # Queue vide, c'est normal
            pass

        
        # Appeler cette fonction régulièrement depuis tkinter
        self.root.after(100, self.process_queue)
    
    def run(self):
        """Lance la boucle tkinter - APPELLE MAINLOOP DANS LE THREAD PRINCIPAL"""
        self.root.mainloop()