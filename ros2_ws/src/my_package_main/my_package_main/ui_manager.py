from .my_node_manager import ManagerNode
from .my_node_manager import Task

import queue
import tkinter as tk
from tkinter import ttk


def parse_vector(text):
    """Convertit 'x;y' en tuple (float, float). Lève ValueError si le format est invalide."""
    x, y = text.split(";")
    return float(x), float(y)

class ManagerUI:
    """Interface graphique, DOIT tourner dans le thread principal"""
    
    def __init__(self, node: ManagerNode):
        self.manager_node = node
        self.root = tk.Tk()
        self.root.title('Fleet Manager Interface')
        self.root.geometry('720x360')
        
        self.robot_labels = {}
        self.setup_ui()
        
        self.update_display()
        
        # Traiter les updates de la queue tous les 100ms
        self.process_queue()

    ##Création des éléments de l'interface
    def setup_ui(self):
        # Titre
        tk.Label(self.root, text="Tasks", font=("Arial", 16, "bold")).pack(pady=10)
        # Tableau
        columns = ("task_id", "robot_id", "state", "pick_up", "delivery")
        headers = ("Task id", "Robot id", "State", "Pick up", "Delivery")
        self.tree = ttk.Treeview(self.root, columns=columns, show="headings", height=8)
        for col, text in zip(columns, headers):
            self.tree.heading(col, text=text)
            self.tree.column(col, width=130, anchor="center")
        self.tree.pack(fill="x", padx=10)

        # Ligne de saisie
        frame = tk.Frame(self.root)
        frame.pack(pady=15)
        self.robot_id_combo = ttk.Combobox(frame, values=[1, 2, 3, 4], state="readonly", width=12,)
        self.robot_id_combo.current(0)
        self.robot_id_combo.pack(side="left", padx=5)
        self.pick_entry = tk.Entry(frame, width=14)     # format "x;y"
        self.pick_entry.pack(side="left", padx=5)
        self.delivery_entry = tk.Entry(frame, width=14)  # format "x;y"
        self.delivery_entry.pack(side="left", padx=5)
        tk.Button(frame, text="Submit", command=self.submit_task).pack(side="left", padx=5)

        tk.Button(frame, text="Ouvrir", command=self.open_window).pack(side="left", padx=5)

    def open_window(self):
        win = tk.Toplevel(self.root)
        win.title("Nouvelle fenêtre")
        win.geometry("300x150")
        tk.Label(win, text="Bonjour depuis la deuxième fenêtre").pack(pady=20)

        canvas = tk.Canvas(win, width=500, height=400, bg="white")
        canvas.pack()
    # Dessins de base (x1, y1, x2, y2 = coin haut-gauche et coin bas-droit)    
        canvas.create_oval(50, 50, 150, 150, outline="blue", fill="lightblue")
        canvas.create_rectangle(200, 50, 350, 130, outline="red", width=2)
        info = tk.Label(win, text="Aucune action").pack()
        tk.Button(win, text="Fermer", command=win.destroy).pack()

        start = {}
        def on_click(event):
            start["x"], start["y"] = event.x, event.y
            info.config(text=f"Clic en ({event.x}, {event.y})")
        def on_drag(event):
            info.config(text=f"Drag en ({event.x}, {event.y})")# Exemple : tracer une ligne pendant le drag
            canvas.create_line(start["x"], start["y"], event.x, event.y, fill="green")
            start["x"], start["y"] = event.x, event.y
        def on_release(event):
            info.config(text=f"Relâché en ({event.x}, {event.y})")
        canvas.bind("<Button-1>", on_click)
        canvas.bind("<B1-Motion>", on_drag)
        canvas.bind("<ButtonRelease-1>", on_release)

    def submit_task(self):
        try:
            robot = int(self.robot_id_combo.get())
            pick_up = parse_vector(self.pick_entry.get())
            delivery = parse_vector(self.delivery_entry.get())
        except ValueError:
            print("Erreur de parsing : format x;y attendu")
            return

        with self.manager_node.lock:
            self.manager_node.add_task(robot=robot,p_x=pick_up[0],p_y=pick_up[1],d_x=delivery[0],d_y=delivery[1])
            #task_id = max(self.manager_node.tasks.keys(), default=-1) + 1
            #self.manager_node.tasks[task_id] = Task(id=task_id, robot=robot, state=0, pickup_x=pick_up[0], pickup_y=pick_up[1], delivery_x=delivery[0], delivery_y=delivery[1])
        
        self.pick_entry.delete(0, tk.END)
        self.delivery_entry.delete(0, tk.END)

        self.update_display()
    
    def update_display(self):
        """Met à jour l'affichage"""
        with self.manager_node.lock:
            all_tasks = list(self.manager_node.tasks.values())

        self.tree.delete(*self.tree.get_children())
        for task in all_tasks:
            self.tree.insert("", "end",
                             values=(task.id, task.robot, task.state, f"({task.pickup_x};{task.pickup_y})",f"({task.delivery_x};{task.delivery_y})",))
        
    
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