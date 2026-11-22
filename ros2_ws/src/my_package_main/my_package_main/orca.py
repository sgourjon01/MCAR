"""
Mathias
Bouchut--Loubet
22/11/2026

Classe qui implémente l'algorithme ORCA pour calculer
les mouvements des robots sans collisions.
"""

from .rvo2_python import RVOSimulator, Vector2

from map import Map


# Distance max pour être considéré comme robot voisin
NEIGHBOR_DIST = 300 # mm

# Nombre maximum de robots voisin considérés
MAX_NEIGHBORS = 5

# Horizon de prédiction pour éviter les autres agents
TIMER_HORIZON = 5 # sec

# Horizon de prédiction pour éviter les obstacles
TIMER_HORIZON_OBST = 5 # sec


class ORCA:
    """
    Classe qui implémente l'algorithme ORCA.
    """

    def __init__(self):
        self.simulator = None
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
            timeHorizonObst=TIMER_HORIZON_OBST
        )

        self.__create_obstacle()

    def __create_obstacle(self):
        """
        Ajoute les obstacles de la carte depuis la classe Map.
        """
        obstacles = []

        for row in range(Map.X):
            for column in range(Map.Y):
                # On traite que les obstacles
                if Map.MAP_TABLE[row, column] != 1:
                    continue

                x0 = column * Map.SIZE
                y0 = (Map.X - 1 - row) * Map.SIZE
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
