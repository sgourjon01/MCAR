"""
    Thomas Eyer 14/11/2026
    
    Ce fichier regroupe les enums contenu par les anciens fichiers task_type.py et robot_state.py.
    Énumérations partagées entre les noeuds Manager et Robot (cf. Topics.md).
    Les classes hérite de IntEnum, classe python typé Int pour des enums.
"""
from enum import IntEnum

# Convention projet : tout agent d'identifiant > 10 est un obstacle statique.
OBSTACLE_ID_MIN = 10


class TaskType(IntEnum):
    """Type de tâche transporté dans l'en-tête du topic `robot_task`.
    """
    ACK = 1
    MOVE_PACKET = 2  # valeur à confirmer avec le Manager
    

class RobotState(IntEnum):
    """État d'un robot, publié sur le topic `robot_state`."""

    WAIT = 1              # en attente, aucune tâche assignée
    SEARCHING_PACKET = 2  # se dirige vers le colis
    TRANSPORTING = 3      # transporte le colis vers sa destination
    TASK_DONE = 4         # tâche terminée, en attente de la suite côté Manager

def is_obstacle_id(agent_id: int) -> bool:
    """True si l'identifiant désigne un obstacle statique (id > 10)."""
    return agent_id > OBSTACLE_ID_MIN