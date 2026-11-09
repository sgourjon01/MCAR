"""
Mathias Bouchut--Loubet
09/11/2026

Classe qui contient la carte.
"""

import numpy as np

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
