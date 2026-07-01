#!/usr/bin/env python3
"""
SystemState enum và bảng transition hợp lệ.
Tất cả thay đổi state phải đi qua StateMachine.
"""

from enum import Enum, auto
from typing import Set, Dict


class SystemState(Enum):
    UNCONFIGURED        = auto()
    IDLE                = auto()
    NAVIGATING          = auto()
    EXECUTING_TASK      = auto()
    RECOVERING          = auto()
    LOW_BATTERY_DOCKING = auto()
    CHARGING            = auto()
    RESUME_AFTER_CHARGE = auto()
    PAUSED              = auto()
    ABORTED             = auto()
    EXPLORING           = auto()


# Transition table: state → tập hợp state tiếp theo hợp lệ
VALID_TRANSITIONS: Dict[SystemState, Set[SystemState]] = {
    SystemState.UNCONFIGURED:        {SystemState.IDLE, SystemState.PAUSED},
    SystemState.IDLE:                {SystemState.NAVIGATING, SystemState.LOW_BATTERY_DOCKING,
                                      SystemState.PAUSED, SystemState.ABORTED, SystemState.EXPLORING},
    SystemState.NAVIGATING:          {SystemState.EXECUTING_TASK, SystemState.RECOVERING,
                                      SystemState.PAUSED, SystemState.LOW_BATTERY_DOCKING},
    SystemState.EXECUTING_TASK:      {SystemState.IDLE, SystemState.RECOVERING,
                                      SystemState.PAUSED, SystemState.LOW_BATTERY_DOCKING},
    SystemState.RECOVERING:          {SystemState.IDLE, SystemState.ABORTED,
                                      SystemState.LOW_BATTERY_DOCKING},
    SystemState.LOW_BATTERY_DOCKING: {SystemState.CHARGING, SystemState.ABORTED},
    SystemState.CHARGING:            {SystemState.RESUME_AFTER_CHARGE},
    SystemState.RESUME_AFTER_CHARGE: {SystemState.IDLE},
    SystemState.PAUSED:              {SystemState.IDLE, SystemState.ABORTED, SystemState.EXPLORING},
    SystemState.ABORTED:             {SystemState.IDLE, SystemState.UNCONFIGURED},
    SystemState.EXPLORING:           {SystemState.PAUSED, SystemState.IDLE,
                                      SystemState.LOW_BATTERY_DOCKING, SystemState.ABORTED},
}