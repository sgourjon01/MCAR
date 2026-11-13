"""
Mathias
Bouchut--Loubet
09/11/2026

Enum qui définit les états des robots.
"""

class RobotState:

    WAITING = 1
    SEARCHING_PACKET = 2
    TRANSPORTING = 3
    TASK_DONE = 4

    STR_STATE = ["None","Waiting","Searching pkg","Transporting pkg","Task done"]
