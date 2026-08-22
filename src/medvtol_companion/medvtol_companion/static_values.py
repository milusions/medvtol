from enum import Enum


class FlightMode(Enum):
    NONE = 0
    OFFBOARD = 1
    TAKEOFF = 2
    LAND = 3
    HOLD = 4
    RTL = 5

    
class FlightStage(Enum):
    IDLE = 0
    TAKEOFF = 1
    CLIMB = 8
    YAW_ALIGN = 9
    CRUISE = 2
    DESCEND = 3
    ALIGN = 4
    LAND = 5
    RTL = 6
    HOLD= 7
    ABORT = 10
    
