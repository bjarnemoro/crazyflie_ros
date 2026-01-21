from enum import IntEnum

class AgentState(IntEnum):
    """Define the three states of the drone."""
    TAKEOFF  = 0
    READY    = 1
    LANDING  = 2

class WorkingMode(IntEnum):
    """Defines if the code is launched from simulation or from the real crazyflie"""
    SIM  = 0
    REAL = 1

class ManagerState(IntEnum):
    WAITING_FOR_ODOMETRY = 0
    READY                = 1
    ERROR                = 2