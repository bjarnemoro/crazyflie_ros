from enum import IntEnum, Enum

class AgentState(IntEnum):
    """Define the three states of the drone."""
    TAKEOFF   = 0
    READY     = 1
    LANDING   = 2
    GATHERING = 3
    STOP      = 4

class WorkingMode(IntEnum):
    """Defines if the code is launched from simulation or from the real crazyflie"""
    SIM  = 0
    REAL = 1

class ManagerState(IntEnum):
    WAITING_FOR_ODOMETRY = 0
    READY                = 1
    ERROR                = 2
    WAITING_FOR_TASKS    = 3

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

    # BOLD colors
    BOLD_BLACK   = "\033[1;30m"
    BOLD_RED     = "\033[1;31m"
    BOLD_GREEN   = "\033[1;32m"
    BOLD_YELLOW  = "\033[1;33m"
    BOLD_BLUE    = "\033[1;34m"
    BOLD_VIOLET  = "\033[1;35m"
    BOLD_CYAN    = "\033[1;36m"
    BOLD_WHITE   = "\033[1;37m"