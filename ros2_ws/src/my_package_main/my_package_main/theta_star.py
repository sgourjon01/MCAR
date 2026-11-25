"""
Mathias
Bouchut--Loubet
22/11/2026

Classe qui implémente l'algorithme Theta Star pour calculer
les points de passage des robots.

Bibliothèque utilisée :
pip install python-motion-planning
https://ai-winter.github.io/python_motion_planning/latest/index.html
"""

from python_motion_planning.common import Grid, TYPES, Visualizer2D
from python_motion_planning.path_planner import ThetaStar as TS

from .map import Map


class ThetaStar:
    """
    Classe qui implémente l'algorithme Theta Star.
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

        # Remplissage à partir des données de Map
        for y in range(Map.Y):
            for x in range(Map.X):
                if Map.MAP_TABLE[y, x] == 1:
                    self.grid[x, y] = TYPES.OBSTACLE
                # else:
                #     self.grid[x, y] = TYPES.FREE

        # Si besoin
        # map_.fill_boundary_with_obstacles() # Obstacles sur les bords
        # map_.inflate_obstacles(radius=2) # Augmente épaisseur obstacles

    def compute_waypoints(self, start: tuple, end: tuple)->list:
        """
        Calcule les points de passage entre le point de départ et
        le point d'arrivée.

        Pre:
            Deux couples d'entiers positifs représentant les points de départ et d'arrivée.
        Post:
            Une liste de couples d'entiers positifs représentant les points de passage.
        """

        # On garde en mémoire pour remettre en place la Grid
        save_start = self.grid[start]
        save_end = self.grid[end]

        # Vérification départ et arrivé pas dans un obstacle
        if save_start == TYPES.OBSTACLE or save_end == TYPES.OBSTACLE:
            return []

        # Mise à jour de la Grid
        self.grid[start] = TYPES.START
        self.grid[end] = TYPES.GOAL

        # Calcul des points de passage
        planner = TS(map_=self.grid, start=start, goal=end)
        path, path_info = planner.plan()

        # Remise en place de la Grid
        self.grid[start] = save_start
        self.grid[end] = save_end

        return path

    def has_line_of_sight(self, a: tuple, b: tuple)->bool:
        """
        Vérifie si deux points peuvent être reliés en ligne droite
        sans rencontrer d'obstacles.
        Algorithme de Bresenham utilisé pour tracer la ligne.

        Pre:
            Deux couples d'entiers positifs représentant les deux points.
        Post:
            Un booléen : vrai si le passage est libre, faux si un obstacle est présent.
        """

        x0, y0 = a
        x1, y1 = b

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        while True:
            # Vérification de la cellule courante
            if self.grid[x0, y0] == TYPES.OBSTACLE:
                return False

            # Toutes les cellules de la ligne testées
            if (x0, y0) == (x1, y1):
                break

            # Cellule suivante (algo de Bresenham)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return True

    def show_debug_map(self, path=None):
        vis = Visualizer2D()
        vis.plot_grid_map(self.grid)
        if path:
            vis.plot_path(path, style="--", color="C4")
        vis.show()
        vis.close()
