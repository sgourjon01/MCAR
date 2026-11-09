from my_node_manager import ManagerNode
from my_node_manager import TaskData

import tkinter as tk
from tkinter import ttk

from dataclasses import dataclass


class ManagerUI:
    """Interface graphique - DOIT tourner dans le thread principal"""
    
    def __init__(self, node: ManagerNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('ROS2 Manager Interface')
        self.root.geometry('700x400')
        
        self.robot_labels = {}
        self.setup_ui()
        
        # Traiter les updates de la queue tous les 100ms
        self.process_queue()
    
    def setup_ui(self):
        """Créer les éléments de l'interface"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        title_label = ttk.Label(
            main_frame,
            text='ROS2 Manager - Robot Status Monitor',
            font=('Arial', 14, 'bold')
        )
        title_label.grid(row=0, column=0, columnspan=5, pady=10)
        
        # En-têtes
        headers = ['Robot ID', 'X', 'Y', 'Theta', 'Velocities', 'State']
        for col, header in enumerate(headers):
            ttk.Label(
                main_frame,
                text=header,
                font=('Arial', 10, 'bold')
            ).grid(row=1, column=col, padx=5, pady=5, sticky=tk.W)
        
        # Lignes pour chaque robot (max 4)
        self.robot_frames = {}
        for robot_num in range(4):
            robot_id = robot_num
            
            id_label = ttk.Label(main_frame, text=f"Robot {robot_id}", width=10)
            id_label.grid(row=2 + robot_num, column=0, padx=5, pady=5)
            
            x_label = ttk.Label(main_frame, text="--", width=10)
            x_label.grid(row=2 + robot_num, column=1, padx=5, pady=5)
            
            y_label = ttk.Label(main_frame, text="--", width=10)
            y_label.grid(row=2 + robot_num, column=2, padx=5, pady=5)
            
            theta_label = ttk.Label(main_frame, text="--", width=10)
            theta_label.grid(row=2 + robot_num, column=3, padx=5, pady=5)
            
            vel_label = ttk.Label(main_frame, text="--", width=25)
            vel_label.grid(row=2 + robot_num, column=4, padx=5, pady=5)
            
            state_label = ttk.Label(main_frame, text="--", width=10)
            state_label.grid(row=2 + robot_num, column=5, padx=5, pady=5)
            
            self.robot_labels[robot_id] = {
                'x': x_label,
                'y': y_label,
                'theta': theta_label,
                'velocities': vel_label,
                'state': state_label
            }
        
        # Status bar
        self.status_label = ttk.Label(
            main_frame,
            text="Waiting for data...",
            font=('Arial', 9),
            foreground='gray')
        self.status_label.grid(row=6, column=0, columnspan=6, pady=10)
        
        # Frame pour ajouter une task
        task_frame = tk.Frame(self.root, bg="#333333")
        #task_frame.pack(pady=10, padx=10, fill=tk.X)
	
        tk.Label(task_frame, text="Ajouter une task", bg="#333333", fg="white", font=("Arial", 10, "bold")).pack()
	
	# Inputs utilisateur
        input_frame = tk.Frame(task_frame, bg="#333333")
        input_frame.pack(pady=5)
	
        tk.Label(input_frame, text="Robot ID:", bg="#333333", fg="white").grid(row=0, column=0, padx=5)
        self.task_robot_id = tk.Entry(input_frame, width=5)
        self.task_robot_id.grid(row=0, column=1, padx=5)
	
        tk.Label(input_frame, text="Pickup (x,y):", bg="#333333", fg="white").grid(row=0, column=2, padx=5)
        self.task_pickup_x = tk.Entry(input_frame, width=8)
        self.task_pickup_x.grid(row=0, column=3, padx=2)
        self.task_pickup_y = tk.Entry(input_frame, width=8)
        self.task_pickup_y.grid(row=0, column=4, padx=2)
	
        tk.Label(input_frame, text="Delivery (x,y):", bg="#333333", fg="white").grid(row=0, column=5, padx=5)
        self.task_delivery_x = tk.Entry(input_frame, width=8)
        self.task_delivery_x.grid(row=0, column=6, padx=2)
        self.task_delivery_y = tk.Entry(input_frame, width=8)
        self.task_delivery_y.grid(row=0, column=7, padx=2)
	
        tk.Button(task_frame, text="Ajouter", bg="#4CAF50", fg="white", command=self.add_task_from_gui).pack(pady=5)

    
    def update_display(self):
        """Met à jour l'affichage"""
        robots = self.node.get_all_robots()
        
        for robot_id in range(4):
            if robot_id in robots:
                pos, etat = robots[robot_id]
                
                self.robot_labels[robot_id]['x'].config(text=f"{pos.x:.2f}")
                self.robot_labels[robot_id]['y'].config(text=f"{pos.y:.2f}")
                self.robot_labels[robot_id]['theta'].config(text=f"{pos.theta:.3f}")
                self.robot_labels[robot_id]['velocities'].config(
                    text=f"vx={pos.vx:.2f} vy={pos.vy:.2f} ω={pos.omega:.2f}"
                )
                self.robot_labels[robot_id]['state'].config(text=f"{etat.etat}")
            else:
                for key in self.robot_labels[robot_id]:
                    self.robot_labels[robot_id][key].config(text="--")
        
        num_robots = len(robots)
        self.status_label.config(text=f"Active robots: {num_robots}/4")
    
    def add_task_from_gui(self):
        try:
            robot_id = int(self.task_robot_id.get())
            pickup_x = float(self.task_pickup_x.get())
            pickup_y = float(self.task_pickup_y.get())
            delivery_x = float(self.task_delivery_x.get())
            delivery_y = float(self.task_delivery_y.get())
        
            task = TaskData(pickup_x, pickup_y, delivery_x, delivery_y, robot_id)
            
            with self.manager_node.task_lock:
                if robot_id not in self.manager_node.tasks:
                    self.manager_node.tasks[robot_id] = []
                self.manager_node.tasks[robot_id].append(task)
        
            # Vider les inputs
            self.task_robot_id.delete(0, tk.END)
            self.task_pickup_x.delete(0, tk.END)
            self.task_pickup_y.delete(0, tk.END)
            self.task_delivery_x.delete(0, tk.END)
            self.task_delivery_y.delete(0, tk.END)
        
            print(f"Task ajoutée au robot {robot_id}")
        except ValueError:
            print("Erreur : vérifiez les valeurs entrées")

    
    def process_queue(self):
        """Traite les messages de la queue ROS2"""
        try:
            while True:
                msg_type, robot_id = self.node.gui_update_queue.get_nowait()
                self.update_display()
        except:
            # Queue vide, c'est normal
            pass
        
        # Appeler cette fonction régulièrement depuis tkinter
        self.root.after(100, self.process_queue)
    
    def run(self):
        """Lance la boucle tkinter - APPELLE MAINLOOP DANS LE THREAD PRINCIPAL"""
        self.root.mainloop()