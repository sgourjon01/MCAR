"""
Mathias
Bouchut--Loubet
09/11/2026

Enumeration pour tous les noeuds.
"""

class RobotState:
    """
    Enum qui définit les états des robots.
    """

    WAIT = 1
    SEARCHING_PACKET = 2
    TRANSPORTING = 3
    TASK_DONE = 4
