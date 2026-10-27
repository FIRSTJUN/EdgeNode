from edgenode_planning.lattice_planner import LatticePlanner


def straight_path():
    return [
        [i * 0.5, 0.0, 0.0]
        for i in range(41)
    ]


def test_clear_path_keeps_center():
    planner = LatticePlanner()

    result = planner.plan(
        straight_path(),
        [],
    )

    assert result is not None
    assert result['status'] == 'CLEAR'
    assert result['selected_offset_m'] == 0.0
    assert result['blocked'] is False


def test_center_obstacle_generates_avoidance():
    planner = LatticePlanner()

    result = planner.plan(
        straight_path(),
        [(8.0, 0.0)],
    )

    assert result is not None
    assert result['status'] == 'AVOIDING'
    assert abs(result['selected_offset_m']) > 0.0
    assert result['blocked'] is False
    assert result['minimum_clearance_m'] >= planner.obstacle_clearance_m


def test_far_obstacle_is_ignored():
    planner = LatticePlanner()

    result = planner.plan(
        straight_path(),
        [(8.0, 5.0)],
    )

    assert result['status'] == 'CLEAR'
    assert result['selected_offset_m'] == 0.0


def test_blocked_when_all_candidates_collide():
    planner = LatticePlanner(
        obstacle_clearance_m=0.8,
    )

    obstacles = [
        (8.0, -1.5),
        (8.0, -0.75),
        (8.0, 0.0),
        (8.0, 0.75),
        (8.0, 1.5),
    ]

    result = planner.plan(
        straight_path(),
        obstacles,
    )

    assert result['status'] == 'BLOCKED'
    assert result['blocked'] is True


def test_avoidance_returns_to_reference_path():
    planner = LatticePlanner()

    result = planner.plan(
        straight_path(),
        [(8.0, 0.0)],
    )

    points = result['points']

    assert abs(points[0][1]) < 1e-6
    assert abs(points[-1][1]) < 1e-6
