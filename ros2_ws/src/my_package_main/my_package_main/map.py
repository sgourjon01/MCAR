"""
Mathias Bouchut--Loubet
09/11/2026

Classe qui contient la carte.
"""

import numpy as np
from math import floor

class Map:

    # Carte de démo
    MAP_TABLE = np.array([
        [1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1]

    ])

    # Taille du tableau X/Y
    X = 6
    Y = 6

    # Correspondance taille réel
    # [mm/cellule]
    SIZE = 100

def obstacle(_x:float, _y:float):
    _X = floor(_x/Map.SIZE)
    _Y = floor(_y/Map.SIZE)
    return _X < 0 or _X > Map.X - 1 or _Y < 0 or _Y > Map.Y - 1 or Map.MAP_TABLE[_X,_Y] == 1