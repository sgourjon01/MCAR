"""
Mathias
Bouchut--Loubet
22/11/2026

Classe qui implémente l'algorithme ORCA pour calculer
les mouvements des robots sans collisions.
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from .rvo2_python import RVOSimulator, Vector2

from .map import Map


# Distance max pour être considéré comme robot voisin
NEIGHBOR_DIST = 300 # mm

# Nombre maximum de robots voisins considérés
MAX_NEIGHBORS = 5

# Horizon de prédiction pour éviter les autres agents
TIMER_HORIZON = 5 # sec

# Horizon de prédiction pour éviter les obstacles
TIMER_HORIZON_OBST = 5 # sec


class ORCA:
    """
    Classe qui implémente l'algorithme ORCA.
    """

    def __init__(self, robot_id: int, robot_radius: float, robot_max_speed: float):
        self.simulator = None
        self.agents = {} # {robot_id : agent_index}

        self.master_id = robot_id
        self.robot_radius = robot_radius
        self.robot_max_speed = robot_max_speed

        self.init_simulator()

    def init_simulator(self):
        """
        Crée et initialise le simulateur ORCA
        """
        self.simulator = RVOSimulator(
            timeStep=0.1, # TODO pas de simulation
            neighborDist=NEIGHBOR_DIST,
            maxNeighbors=MAX_NEIGHBORS,
            timeHorizon=TIMER_HORIZON,
            timeHorizonObst=TIMER_HORIZON_OBST,
            radius=self.robot_radius,
            maxSpeed=self.robot_max_speed
        )

        self.__create_obstacle()

    def __create_obstacle(self):
        """
        Ajoute les obstacles de la carte depuis la classe Map.
        """
        obstacles = []

        for x in range(Map.X):
            for y in range(Map.Y):
                # On ne traite que les obstacles
                if Map.MAP_TABLE[x, y] != 1:
                    continue

                x0 = x * Map.SIZE
                y0 = y * Map.SIZE
                # Sommets dans le sens anti-horaire, en mm
                square = [
                    Vector2(x0, y0),
                    Vector2(x0 + Map.SIZE, y0),
                    Vector2(x0 + Map.SIZE, y0 + Map.SIZE),
                    Vector2(x0, y0 + Map.SIZE),
                ]
                self.simulator.addObstacle(square)
                obstacles.append((x0, y0, Map.SIZE, Map.SIZE))

        self.simulator.processObstacles()

    def update_agent(self, id: int, pos: tuple, speed: tuple):
        """
        Met à jour la pose d'un robot.

        Pre:
            Un entier strictement positif pour l'identifiant du robot et
            deux couples de flottants pour sa position et sa vitesse dans
            le repère du sol en mm et mm/s.
        """
        assert type(id) == int and id > 0

        if id not in self.agents.keys():
            self.agents[id] = self.simulator.addAgent(
                position=Vector2(*pos),
                velocity=Vector2(*speed)
            )
        else:
            self.simulator.setAgentPosition(self.agents[id], Vector2(*pos))
            self.simulator.setAgentVelocity(self.agents[id], Vector2(*speed))

    def compute_optimal_velocity(self, speed_pref: tuple)->tuple:
        """
        Calcule la vitesse optimale au plus proche de la vitesse
        désirée en évitant les autres robots.

        Pre :
            Un couple de flottants représentant la vitesse désirée.
        Post :
            un couple de flottants représentant la vitesse optimale.
        """
        self.simulator.setAgentPrefVelocity(self.agents[self.master_id], Vector2(*speed_pref))

        # Pas de simulation
        self.simulator.doStep()

        # Récupération vitesse optimisée
        speed_opt = self.simulator.getAgentVelocity(self.agents[self.master_id])

        return speed_opt.x_, speed_opt.y_

    def show_debug_map(self):
        """
        Affiche les obstacles et les robots présents dans le simulateur.
        """


        simulator = self.simulator
        if simulator is None:
            raise RuntimeError("Le simulateur ORCA n'est pas initialisé.")

        fig, ax = plt.subplots()
        bounds_x = []
        bounds_y = []

        # Les obstacles sont stockés par sommets, chaque sommet pointant vers
        # le suivant dans son polygone.
        for vertex_id in range(len(simulator.obstacles_)):
            vertex = simulator.getObstacleVertex(vertex_id)
            next_id = simulator.getNextObstacleVertexNo(vertex_id)
            next_vertex = simulator.getObstacleVertex(next_id)
            ax.plot(
                [vertex.x_, next_vertex.x_],
                [vertex.y_, next_vertex.y_],
                color="dimgray",
                linewidth=2,
                label="Obstacle" if vertex_id == 0 else None,
            )
            bounds_x.append(vertex.x_)
            bounds_y.append(vertex.y_)

        for agent_id in range(simulator.getNumAgents()):
            position = simulator.getAgentPosition(agent_id)
            radius = simulator.getAgentRadius(agent_id)
            ax.add_patch(Circle(
                (position.x_, position.y_),
                radius,
                color="tab:blue",
                alpha=0.65,
                label="Robot" if agent_id == 0 else None,
            ))
            ax.plot(position.x_, position.y_, ".", color="black", markersize=3)
            bounds_x.extend((position.x_ - radius, position.x_ + radius))
            bounds_y.extend((position.y_ - radius, position.y_ + radius))

        if bounds_x and bounds_y:
            margin = max(max(bounds_x) - min(bounds_x), max(bounds_y) - min(bounds_y)) * 0.05
            margin = max(margin, 1.0)
            ax.set_xlim(min(bounds_x) - margin, max(bounds_x) + margin)
            ax.set_ylim(min(bounds_y) - margin, max(bounds_y) + margin)

        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_title(f"État du simulateur ORCA (t={simulator.getGlobalTime():.2f} s)")
        if simulator.obstacles_ or simulator.getNumAgents():
            ax.legend()
        ax.grid(True, alpha=0.3)
        plt.show()
