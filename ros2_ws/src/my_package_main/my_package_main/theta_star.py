"""
Mathias
Bouchut--Loubet
22/11/2026

Classe qui implémente l'algorithme Theta Star pour
calculer les points de passages des robots.

Lib utilisé :
pip install python-motion-planning
https://ai-winter.github.io/python_motion_planning/latest/index.html
"""

from python_motion_planning.common import Grid, TYPES
from python_motion_planning.path_planner import ThetaStar as TS

from .map import Map


class ThetaStar:
    """
    Classe qui implémente Theta Star.
    """

    def __init__(self):
        self.grid = None
        self.generate_grid()

    def generate_grid(self):
        """
        Génère la grille représentant la carte à partir de
        la classe Map.
        """
        # Création de la Grid
        self.grid = Grid(bounds=[[0, Map.X], [0, Map.Y]])

        # Remplissage avec les données de Map
        for y in range(Map.Y):
            for x in range(Map.X):
                if Map.MAP_TABLE[y, x] == 1:
                    self.grid[x, y] = TYPES.OBSTACLE
                # else:
                #     self.grid[x, y] = TYPES.FREE

        # Si besoin
        # map_.fill_boundary_with_obstacles() # Obstacles sur les bords
        # map_.inflate_obstacles(radius=2) # Augmente épaisseurs obstacles
