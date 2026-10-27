from edgenode_planning.planning_fsm import PlanningFSM


def update_ready_fsm(fsm=None, **overrides):
    if fsm is None:
        fsm = PlanningFSM()

    values = dict(
        localization_ready=True,
        map_match_ready=True,
        global_path_ready=True,
        local_path_ready=True,
        speed_ready=True,
        drive_status='DISABLED',
        obstacle_avoid_requested=False,
        traffic_stop_requested=False,
        emergency_stop_requested=False,
        goal_reached=False,
    )

    values.update(overrides)
    return fsm, fsm.update(**values)


def test_initial_state():
    fsm = PlanningFSM()
    assert fsm.state == PlanningFSM.WAIT_FOR_LOCALIZATION
    assert fsm.motion_allowed is False


def test_wait_for_localization():
    fsm, state = update_ready_fsm(
        localization_ready=False,
    )
    assert state == PlanningFSM.WAIT_FOR_LOCALIZATION


def test_wait_for_map_match():
    fsm, state = update_ready_fsm(
        map_match_ready=False,
    )
    assert state == PlanningFSM.WAIT_FOR_MAP_MATCH


def test_wait_for_global_path():
    fsm, state = update_ready_fsm(
        global_path_ready=False,
    )
    assert state == PlanningFSM.WAIT_FOR_GLOBAL_PATH


def test_wait_for_local_path():
    fsm, state = update_ready_fsm(
        local_path_ready=False,
    )
    assert state == PlanningFSM.WAIT_FOR_LOCAL_PATH


def test_wait_for_local_path_when_speed_invalid():
    fsm, state = update_ready_fsm(
        speed_ready=False,
    )
    assert state == PlanningFSM.WAIT_FOR_LOCAL_PATH


def test_ready():
    fsm, state = update_ready_fsm(
        drive_status='DISABLED',
    )
    assert state == PlanningFSM.READY
    assert fsm.motion_allowed is False


def test_normal_drive():
    fsm, state = update_ready_fsm(
        drive_status='ACTIVE',
    )
    assert state == PlanningFSM.NORMAL_DRIVE
    assert fsm.motion_allowed is True


def test_obstacle_avoid():
    fsm, state = update_ready_fsm(
        drive_status='ACTIVE',
        obstacle_avoid_requested=True,
    )
    assert state == PlanningFSM.OBSTACLE_AVOID
    assert fsm.motion_allowed is True


def test_traffic_stop():
    fsm, state = update_ready_fsm(
        drive_status='ACTIVE',
        traffic_stop_requested=True,
    )
    assert state == PlanningFSM.TRAFFIC_STOP
    assert fsm.motion_allowed is False
    assert fsm.stop_required is True


def test_goal_reached():
    fsm, state = update_ready_fsm(
        drive_status='ACTIVE',
        goal_reached=True,
    )
    assert state == PlanningFSM.GOAL_REACHED
    assert fsm.motion_allowed is False


def test_safety_stop():
    fsm, state = update_ready_fsm(
        drive_status='EXPIRED',
    )
    assert state == PlanningFSM.SAFETY_STOP
    assert fsm.motion_allowed is False


def test_emergency_stop():
    fsm, state = update_ready_fsm(
        drive_status='ACTIVE',
        emergency_stop_requested=True,
    )
    assert state == PlanningFSM.EMERGENCY_STOP
    assert fsm.motion_allowed is False


def test_error():
    fsm, state = update_ready_fsm(
        drive_status='INVALID_CONFIG',
    )
    assert state == PlanningFSM.ERROR
    assert fsm.motion_allowed is False


def test_traffic_stop_has_priority_over_obstacle_avoid():
    fsm, state = update_ready_fsm(
        drive_status='ACTIVE',
        obstacle_avoid_requested=True,
        traffic_stop_requested=True,
    )
    assert state == PlanningFSM.TRAFFIC_STOP


def test_emergency_has_highest_priority():
    fsm, state = update_ready_fsm(
        localization_ready=False,
        drive_status='EXPIRED',
        obstacle_avoid_requested=True,
        traffic_stop_requested=True,
        emergency_stop_requested=True,
        goal_reached=True,
    )
    assert state == PlanningFSM.EMERGENCY_STOP


def test_goal_reached_blocks_motion():
    fsm, state = update_ready_fsm(
        drive_status='ACTIVE',
        obstacle_avoid_requested=True,
        goal_reached=True,
    )
    assert state == PlanningFSM.GOAL_REACHED
    assert fsm.motion_allowed is False


def test_recovery_from_obstacle_avoid():
    fsm = PlanningFSM()

    _, state = update_ready_fsm(
        fsm,
        drive_status='ACTIVE',
        obstacle_avoid_requested=True,
    )
    assert state == PlanningFSM.OBSTACLE_AVOID

    _, state = update_ready_fsm(
        fsm,
        drive_status='ACTIVE',
        obstacle_avoid_requested=False,
    )
    assert state == PlanningFSM.NORMAL_DRIVE


def test_recovery_from_traffic_stop():
    fsm = PlanningFSM()

    _, state = update_ready_fsm(
        fsm,
        drive_status='ACTIVE',
        traffic_stop_requested=True,
    )
    assert state == PlanningFSM.TRAFFIC_STOP

    _, state = update_ready_fsm(
        fsm,
        drive_status='ACTIVE',
        traffic_stop_requested=False,
    )
    assert state == PlanningFSM.NORMAL_DRIVE


def test_previous_state_tracking():
    fsm = PlanningFSM()

    _, state = update_ready_fsm(
        fsm,
        drive_status='DISABLED',
    )

    assert state == PlanningFSM.READY
    assert fsm.previous_state == PlanningFSM.WAIT_FOR_LOCALIZATION

    _, state = update_ready_fsm(
        fsm,
        drive_status='ACTIVE',
    )

    assert state == PlanningFSM.NORMAL_DRIVE
    assert fsm.previous_state == PlanningFSM.READY


def test_reset():
    fsm = PlanningFSM()

    update_ready_fsm(
        fsm,
        drive_status='ACTIVE',
    )

    assert fsm.state == PlanningFSM.NORMAL_DRIVE

    state = fsm.reset()

    assert state == PlanningFSM.WAIT_FOR_LOCALIZATION
    assert fsm.motion_allowed is False
