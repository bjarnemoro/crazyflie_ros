from enum import IntEnum, Enum

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

class AnsiColor(str, Enum):
    RESET = "\033[0m"

    # Regular colors
    BLACK   = "\033[30m"
    RED     = "\033[31m"
    GREEN   = "\033[32m"
    YELLOW  = "\033[33m"
    BLUE    = "\033[34m"
    VIOLET  = "\033[35m"  # aka magenta
    CYAN    = "\033[36m"
    WHITE   = "\033[37m"

    # Bright colors
    BRIGHT_BLACK   = "\033[90m"
    BRIGHT_RED     = "\033[91m"
    BRIGHT_GREEN   = "\033[92m"
    BRIGHT_YELLOW  = "\033[93m"
    BRIGHT_BLUE    = "\033[94m"
    BRIGHT_VIOLET  = "\033[95m"
    BRIGHT_CYAN    = "\033[96m"
    BRIGHT_WHITE   = "\033[97m"