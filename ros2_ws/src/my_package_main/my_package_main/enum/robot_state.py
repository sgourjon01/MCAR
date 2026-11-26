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

    WAITING = 1
    SEARCHING_PACKET = 2
    TRANSPORTING = 3
    TASK_DONE = 4

    STR = ["None","Waiting","Searching pkg","Transporting pkg","Task done"]
    COLORS = ["white", "red", "gold2", "green", "blue"]