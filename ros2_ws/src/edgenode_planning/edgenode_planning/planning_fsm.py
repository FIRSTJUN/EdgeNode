class PlanningFSM:
    """High-level planning finite state machine.

    This FSM decides the high-level driving behavior.
    It does not generate paths or steering commands itself.

    Priority:
        EMERGENCY_STOP
        ERROR
        SAFETY_STOP
        readiness checks
        GOAL_REACHED
        READY
        TRAFFIC_STOP
        OBSTACLE_AVOID
        NORMAL_DRIVE
    """

    # --------------------------------------------------
    # System readiness states
    # --------------------------------------------------
    WAIT_FOR_LOCALIZATION = 'WAIT_FOR_LOCALIZATION'
    WAIT_FOR_MAP_MATCH = 'WAIT_FOR_MAP_MATCH'
    WAIT_FOR_GLOBAL_PATH = 'WAIT_FOR_GLOBAL_PATH'
    WAIT_FOR_LOCAL_PATH = 'WAIT_FOR_LOCAL_PATH'

    # --------------------------------------------------
    # Ready / normal driving
    # --------------------------------------------------
    READY = 'READY'
    NORMAL_DRIVE = 'NORMAL_DRIVE'

    # --------------------------------------------------
    # Driving behavior states
    # --------------------------------------------------
    OBSTACLE_AVOID = 'OBSTACLE_AVOID'

    # --------------------------------------------------
    # Stop states
    # --------------------------------------------------
    TRAFFIC_STOP = 'TRAFFIC_STOP'
    GOAL_REACHED = 'GOAL_REACHED'
    SAFETY_STOP = 'SAFETY_STOP'
    EMERGENCY_STOP = 'EMERGENCY_STOP'

    # --------------------------------------------------
    # Fault state
    # --------------------------------------------------
    ERROR = 'ERROR'

    # Temporary compatibility alias.
    DRIVING = NORMAL_DRIVE

    MOTION_STATES = (
        NORMAL_DRIVE,
        OBSTACLE_AVOID,
    )

    STOP_STATES = (
        TRAFFIC_STOP,
        GOAL_REACHED,
        SAFETY_STOP,
        EMERGENCY_STOP,
        ERROR,
    )

    def __init__(self):
        self.state = self.WAIT_FOR_LOCALIZATION
        self.previous_state = self.state

    def reset(self):
        """Reset FSM to its initial safe state."""
        self.previous_state = self.state
        self.state = self.WAIT_FOR_LOCALIZATION
        return self.state

    def update(
        self,
        *,
        localization_ready,
        map_match_ready,
        global_path_ready,
        local_path_ready,
        speed_ready,
        drive_status,
        obstacle_avoid_requested=False,
        traffic_stop_requested=False,
        emergency_stop_requested=False,
        goal_reached=False,
    ):
        """Evaluate all inputs and return the new FSM state."""

        # --------------------------------------------------
        # 1. Highest-priority stop / fault conditions
        # --------------------------------------------------
        if emergency_stop_requested:
            next_state = self.EMERGENCY_STOP

        elif drive_status == 'INVALID_CONFIG':
            next_state = self.ERROR

        elif drive_status == 'EXPIRED':
            next_state = self.SAFETY_STOP

        # --------------------------------------------------
        # 2. Planning readiness
        # --------------------------------------------------
        elif not localization_ready:
            next_state = self.WAIT_FOR_LOCALIZATION

        elif not map_match_ready:
            next_state = self.WAIT_FOR_MAP_MATCH

        elif not global_path_ready:
            next_state = self.WAIT_FOR_GLOBAL_PATH

        elif not local_path_ready or not speed_ready:
            next_state = self.WAIT_FOR_LOCAL_PATH

        # --------------------------------------------------
        # 3. Mission completion
        # --------------------------------------------------
        elif goal_reached:
            next_state = self.GOAL_REACHED

        # --------------------------------------------------
        # 4. Vehicle not armed
        # --------------------------------------------------
        elif drive_status != 'ACTIVE':
            next_state = self.READY

        # --------------------------------------------------
        # 5. Active driving behavior arbitration
        # --------------------------------------------------
        elif traffic_stop_requested:
            next_state = self.TRAFFIC_STOP

        elif obstacle_avoid_requested:
            next_state = self.OBSTACLE_AVOID

        else:
            next_state = self.NORMAL_DRIVE

        # --------------------------------------------------
        # 6. Transition bookkeeping
        # --------------------------------------------------
        if next_state != self.state:
            self.previous_state = self.state
            self.state = next_state

        return self.state

    @property
    def motion_allowed(self):
        """True only when the FSM permits forward vehicle motion."""
        return self.state in self.MOTION_STATES

    @property
    def stop_required(self):
        """True when the current state explicitly requires zero target speed."""
        return self.state in self.STOP_STATES
