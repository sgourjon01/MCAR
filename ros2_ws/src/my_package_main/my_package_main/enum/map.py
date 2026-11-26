"""
Mathias Bouchut--Loubet
09/11/2026

Constantes de la carte pour tous les noeuds.
"""

import numpy as np

class Map:
    """
    Classe qui contient les infos de la carte.
    """

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
    X, Y = MAP_TABLE.shape

    # Correspondance taille réel
    # [mm/cellule]
    SIZE = 100
